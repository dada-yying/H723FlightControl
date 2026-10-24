"""CMSIS-DAP capture for the critical-section profiler snapshot (DAPLink probes).

Subcommands:
  watch    Attach WITHOUT halting or resetting; print live header fields once a second.
           Safe during scenario runs: only AHB reads, CPU keeps running.
  capture  Attach, then issue the FINAL halt (collection endpoint), and dump
           snapshot/headers/guards/heap/handles/stacks into the capture directory.
           The target is LEFT HALTED afterwards; no resume, no reset, no flash write.
  flash    Program a HEX (used to install measurement firmware and to restore
           production firmware afterwards). This DOES reset the target.

File names match what snapshot_tool.py decode expects.

Examples (run from project root):
  python measurement/snapshot_export/dap_capture.py watch \
      --manifest audit/dwt_export/bundle/manifest.json
  python measurement/snapshot_export/dap_capture.py capture \
      --manifest audit/dwt_export/bundle/manifest.json --capture audit/dwt_capture_run1
  python measurement/snapshot_export/dap_capture.py flash \
      --hex audit/dwt_profiler/measurement_ac5/firmware.hex
"""
from pathlib import Path
import argparse
import json
import struct
import sys
import time

DTCM_LO = 0x20000000
DTCM_HI = 0x20020000  # 128 KiB DTCM, exclusive upper bound
HEADER_WORDS = ('version', 'sequence', 'complete', 'valid', 'faults',
                'core_clock_hz', 'site_count', 'current_depth', 'max_depth',
                'startup_pairs', 'outer_hits', 'max_outer_body_cycles',
                'max_outer_envelope_cycles', 'max_outer_body_us_ceil',
                'max_outer_envelope_us_ceil', 'max_accounting_body_cycles',
                'max_snapshot_envelope_cycles', 'measurement_rc_stack_bytes',
                'catalog')


def choose_unique_id():
    """Enumerate probes once and pick the unique one. Explicitly passing the
    unique id to ConnectHelper avoids its auto-select path reusing an interface
    object that was already opened during enumeration."""
    from pyocd.probe.pydapaccess import DAPAccess
    probes = DAPAccess.get_connected_devices()
    if not probes:
        sys.exit('No CMSIS-DAP probe found. Check the USB cable, close Keil, '
                 'and re-plug the probe.')
    if len(probes) > 1:
        sys.exit('Multiple probes found; edit this script to pass unique_id: '
                 + ', '.join(p.get_unique_id() for p in probes))
    return probes[0].get_unique_id()


def open_session(connect_mode):
    from pyocd.core.helpers import ConnectHelper
    unique_id = choose_unique_id()
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=unique_id,
        target_override='stm32h723xx',
        connect_mode=connect_mode,
        options={'frequency': 2000000, 'logging.level': 'warning'})
    if session is None:
        sys.exit('Could not open probe %s. Close Keil/other debuggers and re-plug.' % unique_id)
    # pyocd 0.46: constructing the Session briefly opens the probe
    # (_TemporaryOpen) to read board info. If the probe firmware is wedged
    # (e.g. a previous process died while the probe was open), the identify
    # exchange times out; the error is swallowed while the USB interface is
    # left half-open, so session.open() then dies with a bare AssertionError
    # ("assert self.closed is True"). Detect that state and say what to do.
    link = session.probe._link
    if not getattr(link, '_is_open', False) and not getattr(link._interface, 'closed', True):
        try:
            link._interface.close()
        except Exception:
            pass
        sys.exit('Probe %s is unresponsive (CMSIS-DAP identify timed out; USB '
                 'interface left half-open). Re-plug the probe USB cable, make '
                 'sure no other debugger (Keil) is attached, then retry.' % unique_id)
    session.open()
    return session


def read_header(target, addr):
    raw = bytes(target.read_memory_block8(addr, 4 * len(HEADER_WORDS)))
    return dict(zip(HEADER_WORDS, struct.unpack('<%dI' % len(HEADER_WORDS), raw))), raw


def write_bin(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def cmd_watch(manifest):
    sess = open_session('attach')
    target = sess.target
    addr = manifest['snapshot_address']
    print('Attached (no reset, no halt). Watching g_cp_snapshot @ 0x%08X. Ctrl+C to stop.' % addr)
    print('%-10s %-8s %-8s %-6s %-7s %-7s %-9s %-12s' %
          ('sequence', 'complete', 'valid', 'faults', 'depth', 'maxDepth', 'outerHits', 'maxOuterUs'))
    try:
        while True:
            h, _ = read_header(target, addr)
            print('%-10d %-8d %-8d 0x%-5x %-7d %-7d %-9d body=%-4d env=%-4d' % (
                h['sequence'], h['complete'], h['valid'], h['faults'],
                h['current_depth'], h['max_depth'], h['outer_hits'],
                h['max_outer_body_us_ceil'], h['max_outer_envelope_us_ceil']))
            time.sleep(1.0)
    except KeyboardInterrupt:
        print('\nDetaching; target keeps running.')
    finally:
        sess.close()


def check_dtcm(addr, size, what):
    if not (DTCM_LO <= addr and addr + size <= DTCM_HI):
        sys.exit('Out-of-DTCM range for %s: 0x%08X + %d' % (what, addr, size))


def cmd_capture(manifest, capture_dir):
    syms = manifest['symbols']
    addr = manifest['snapshot_address']
    size = manifest['snapshot_size']
    hdr = manifest['header_bytes']
    tcb_off = manifest['tcb_pxStack_offset']

    sess = open_session('attach')
    try:
        target = sess.target
        # Pre-halt live sequence (AHB read, CPU still running).
        pre, _ = read_header(target, addr)
        print('Live before halt: sequence=%d valid=%d faults=0x%x depth=%d' % (
            pre['sequence'], pre['valid'], pre['faults'], pre['current_depth']))
        print('Halting (final collection endpoint)...')
        # pyocd 0.46 SoCTarget.halt() takes no timeout and only writes DHCSR;
        # poll S_HALT ourselves to preserve the original 3 s budget.
        target.halt()
        deadline = time.time() + 3.0
        while time.time() < deadline and not target.is_halted():
            time.sleep(0.05)
        if not target.is_halted():
            sys.exit('Halt failed; no bytes dumped. Retry with a fresh free run.')
        time.sleep(0.05)

        def dump(name, a, n):
            check_dtcm(a, n, name)
            b = bytes(target.read_memory_block8(a, n))
            if len(b) != n:
                sys.exit('Short read for %s: %d/%d' % (name, len(b), n))
            write_bin(capture_dir / name, b)
            return b

        header_before = dump('header_before.bin', addr, hdr)
        snapshot = dump('snapshot.bin', addr, size)
        header_after = dump('header_after.bin', addr, hdr)
        if header_before != snapshot[:hdr] or header_after != snapshot[:hdr]:
            print('WARNING: header mismatch across dumps; captured anyway for forensics, do not accept.')
        else:
            print('Header consistent across before/full/after dumps.')

        dump('live_faults.bin', syms['s_faults'], 4)
        dump('live_depth.bin', syms['s_depth'], 4)
        dump('heap_min.bin', syms['xMinimumEverFreeBytesRemaining'], 4)
        dump('heap_free.bin', syms['xFreeBytesRemaining'], 4)

        for t in manifest['tasks']:
            key = t['handle_symbol']
            haddr = t['handle_address']
            handle = dump(key + '.handle.bin', haddr, 4)
            tcb = struct.unpack('<I', handle)[0]
            check_dtcm(tcb, tcb_off + 4, key + ' TCB')
            tcb_prefix = dump(key + '.tcb.bin', tcb, tcb_off + 4)
            stack_base = struct.unpack_from('<I', tcb_prefix, tcb_off)[0]
            stack_size = t['measurement_stack_bytes']
            check_dtcm(stack_base, stack_size, key + ' stack')
            dump(key + '.stack.bin', stack_base, stack_size)
            print('  %-28s TCB=0x%08X stackBase=0x%08X size=%d' % (key, tcb, stack_base, stack_size))
    finally:
        # Always release the probe cleanly: a process killed while the probe is
        # open wedges its firmware until USB re-plug. Closing the session never
        # resumes/resets the core, so a successful capture is left HALTED.
        sess.close()
    post = dict(zip(HEADER_WORDS, struct.unpack_from('<%dI' % len(HEADER_WORDS), snapshot)))
    meta = dict(captured_at=time.strftime('%Y-%m-%d %H:%M:%S'),
                pre_halt_sequence=pre['sequence'],
                halted_sequence=post['sequence'],
                snapshot_sha256=__import__('hashlib').sha256(snapshot).hexdigest(),
                target_left='HALTED', reset_or_resume=False,
                note='Decode with: python measurement/snapshot_export/snapshot_tool.py '
                     'decode --manifest audit/dwt_export/bundle/manifest.json --capture <dir>')
    write_bin(capture_dir / 'capture_info.json', json.dumps(meta, indent=2).encode('utf-8'))
    print('\nCapture complete in %s' % capture_dir)
    print('Target is HALTED. Next: decode, then power-cycle / reflash production.')


def cmd_flash(hex_path):
    from pyocd.flash.file_programmer import FileProgrammer
    # IMPORTANT: the Horco probe is wired with four SWD lines only
    # (SWDIO/SWCLK/GND/3V3, no nRST). On STM32H7 the 'halt' connect mode
    # asserts the physical nRESET line before DP init and immediately reads it
    # back (target_STM32H723xx.safe_reset_and_halt starts with
    # `assert self.dp.is_reset_asserted()`); without an nRST wire the read-back
    # stays high and the assert raises AssertionError. Attach without any
    # reset instead. The flash loader halts the core itself (flash.py init,
    # reset=False -> target.halt()), and the post-program reset uses
    # SYSRESETREQ software reset (the H723 default reset type), which needs no
    # nRST wire. If an nRST wire is ever added, 'halt' can be restored here.
    sess = open_session('attach')
    try:
        target = sess.target
        # pyocd 0.46 halt() takes no timeout; poll S_HALT ourselves.
        target.halt()
        deadline = time.time() + 3.0
        while time.time() < deadline and not target.is_halted():
            time.sleep(0.05)
        if not target.is_halted():
            sess.close()
            sys.exit('Could not halt core before flashing. No bytes written.')
        FileProgrammer(sess).program(str(hex_path), format='hex')
        print('Programmed %s' % hex_path)
        target.reset_and_resume()
        print('Reset and running (SYSRESETREQ).')
    finally:
        sess.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--manifest', type=Path, required=True)
    p = sub.add_parser('watch', parents=[common])
    p = sub.add_parser('capture', parents=[common])
    p.add_argument('--capture', type=Path, required=True)
    p = sub.add_parser('flash')
    p.add_argument('--hex', type=Path, required=True)
    a = ap.parse_args()

    if a.cmd == 'flash':
        cmd_flash(a.hex.resolve())
        return
    manifest = json.loads(a.manifest.read_text(encoding='utf-8'))
    if a.cmd == 'watch':
        cmd_watch(manifest)
    else:
        cmd_capture(manifest, a.capture.resolve())


if __name__ == '__main__':
    main()
