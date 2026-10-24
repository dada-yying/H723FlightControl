"""Independent AC5 measurement build and exact preprocessor coverage audit.
No production source, project, scatter file or baseline artifact is rewritten.
"""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
MODULE = Path(__file__).resolve().parent
MDK = ROOT / 'MDK-ARM'
BIN = Path(r'D:\keil5\ARM\ARMCC\Bin')


def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(text.replace('\r\n', '\n').replace('\n', '\r\n').encode('utf-8'))


def hash_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scrub(source):
    # Keep line/byte positions so macro call line numbers remain exact.
    pattern = r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\x27(?:\\.|[^\x27\\])*\x27'
    return re.sub(pattern, lambda m: ''.join('\n' if c == '\n' else ' ' for c in m[0]), source, flags=re.S)


def scan(path):
    s = scrub(path.read_text(encoding='utf-8'))
    functions = []
    pattern = r'(?m)^[ \t]*(?:static[ \t]+)?(?:[A-Za-z_]\w*[ \t*]+)+([A-Za-z_]\w*)[ \t]*\([^;{}]*\)[ \t\r\n]*\{'
    for m in re.finditer(pattern, s):
        if m[1] in ('if', 'for', 'while', 'switch'): continue
        start = m.end() - 1
        depth, end = 1, start + 1
        while depth and end < len(s):
            depth += (s[end] == '{') - (s[end] == '}')
            end += 1
        functions.append((start, end, m[1]))
    calls = []
    for m in re.finditer(r'\btask(ENTER|EXIT)_CRITICAL\s*\(\s*\)\s*;', s):
        owner = [f for f in functions if f[0] < m.start() < f[1]]
        if len(owner) != 1: raise RuntimeError(f'Unresolved function at {path}:{s[:m.start()].count(chr(10))+1}')
        calls.append(dict(kind=m[1], line=s[:m.start()].count('\n')+1, function=owner[0][2]))
    return calls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', default='audit/dwt_profiler/measurement_ac5')
    args = ap.parse_args()
    out = (ROOT / args.output).resolve()
    if not out.is_relative_to(ROOT / 'audit'):
        raise RuntimeError('Measurement artifacts must remain under project audit/')
    out.mkdir(parents=True, exist_ok=True)
    generated = out / 'generated'
    generated.mkdir(exist_ok=True)
    project = ET.parse(MDK/'CtrlBoard-H7.uvprojx')
    ctl = project.find('.//TargetArmAds/Cads/VariousControls')
    cc = ['--cpu=Cortex-M7.fp.dp', '--c99', '--gnu', '-O3', '--split_sections', '-g']
    for inc in ctl.findtext('IncludePath').split(';'): cc += ['-I', inc.strip()]
    for define in ctl.findtext('Define').split(','): cc += ['-D', define.strip()]
    cc += ['-I', str(MODULE), '-I', str(generated)]
    inputs = [(MDK / f.findtext('FilePath').replace('\\', '/')).resolve()
              for f in project.findall('.//Groups/Group/Files/File') if f.findtext('FileType') in ('1', '2')]
    inputs = list(dict.fromkeys(inputs))
    tracked = [p for folder in ('App', 'BSP', 'Core', 'Devices', 'Drivers', 'Middlewares', 'ThirdParty', 'MDK-ARM')
               for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix.lower() in ('.c', '.h', '.s', '.sct', '.uvprojx', '.ps1')]
    before = {p.relative_to(ROOT).as_posix(): hash_file(p) for p in tracked}
    save(out/'production_hashes_before.json', json.dumps(before, indent=2))
    sites, headers, audit = [], {}, []
    for path in inputs:
        rel = path.relative_to(ROOT).as_posix()
        if path.suffix.lower() != '.c' or rel.split('/')[0] not in ('App', 'Core', 'Devices', 'BSP'): continue
        calls = scan(path)
        if not calls and rel != 'Core/Src/freertos.c': continue
        mapping = {}
        for function in dict.fromkeys(c['function'] for c in calls):
            group = [c for c in calls if c['function'] == function]
            entries = [c for c in group if c['kind'] == 'ENTER']
            exits = [c for c in group if c['kind'] == 'EXIT']
            if len(entries) != 1 or not exits:
                raise RuntimeError(f'Needs explicit multi-entry CFG mapping: {rel}:{function}')
            sid = len(sites)
            site = dict(id=sid, file=rel, function=function, entry_line=entries[0]['line'],
                        exit_lines=[c['line'] for c in exits], runtime='NOT MEASURED')
            sites.append(site)
            for c in group: mapping[c['line']] = sid
        header = generated / (path.stem + '_forced.h')
        text = '#include "FreeRTOS.h"\n#include "task.h"\n#include "critical_profiler.h"\n'
        if rel == 'Core/Src/freertos.c':
            text += '#include "vofa.h"\nHAL_StatusTypeDef Cp_TelemetryTap(const uint8_t frame[VOFA_FLIGHT_FRAME_SIZE]);\n#define VOFA_SendFrame(frame) Cp_TelemetryTap(frame)\n'
            text += '#include "cmsis_os2.h"\nosThreadId_t Cp_ThreadNew(osThreadFunc_t function, void *argument, const osThreadAttr_t *attr);\n#define osThreadNew(function,argument,attr) Cp_ThreadNew(function,argument,attr)\n'
        text += '#define CP_JOIN_INNER(a,b) a##b\n#define CP_JOIN(a,b) CP_JOIN_INNER(a,b)\n'
        for line, sid in mapping.items(): text += f'#define CP_SITE_LINE_{line} {sid}U\n'
        text += '#undef taskENTER_CRITICAL\n#undef taskEXIT_CRITICAL\n'
        text += '#define taskENTER_CRITICAL() Cp_Enter(CP_JOIN(CP_SITE_LINE_,__LINE__))\n'
        text += '#define taskEXIT_CRITICAL() Cp_Exit(CP_JOIN(CP_SITE_LINE_,__LINE__))\n'
        save(header, text)
        headers[path] = header
        audit.append(dict(file=rel, expected_calls=calls, preinclude=str(header)))
    save(generated/'cp_sites.h', f'#ifndef CP_SITES_H\n#define CP_SITES_H\n#define CP_SITE_COUNT {len(sites)}U\n#endif\n')
    catalog = '#include "critical_profiler.h"\nconst CpSiteDescriptor g_cp_sites[CP_SITE_COUNT] = {\n'
    for s in sites: catalog += '    {"%s", "%s", %dU},\n' % (s['file'], s['function'], s['entry_line'])
    save(generated/'cp_sites.c', catalog+'};\n')
    save(out/'coverage.json', json.dumps(sites, indent=2))
    transcript, commands = [], []

    def run(argv, artifact=None):
        commands.append(argv)
        p = subprocess.run(argv, cwd=MDK, capture_output=True, timeout=120)
        stdout, stderr = p.stdout.decode('utf-8', 'replace'), p.stderr.decode('utf-8', 'replace')
        if artifact: artifact.parent.mkdir(parents=True, exist_ok=True); artifact.write_bytes(p.stdout)
        else: transcript.append(stdout)
        transcript.append(stderr)
        save(out/'build.log', '\n'.join(transcript))
        save(out/'commands.json', json.dumps(commands, indent=2))
        if p.returncode: raise RuntimeError(f'Command failed: {argv}\n{stderr}')
        return stdout

    objects = []
    for path in inputs + [MODULE/'critical_profiler.c', MODULE/'telemetry_adapter.c', generated/'cp_sites.c']:
        obj = out / (path.stem+'.o')
        if obj in objects: raise RuntimeError('Duplicate object name')
        if path.suffix.lower() == '.s':
            run([str(BIN/'armasm.exe'), '--cpu', 'Cortex-M7.fp.dp', '-g', str(path), '-o', str(obj)])
        else:
            options = cc + (['--preinclude', str(headers[path])] if path in headers else [])
            if path in headers:
                pp = out/'preprocessed'/(path.stem+'.i')
                text = run([str(BIN/'armcc.exe'), *options, '-E', str(path)], pp)
                expected = [s for s in sites if s['file'] == path.relative_to(ROOT).as_posix()]
                actual_enter = collections.Counter(int(x) for x in re.findall(r'Cp_Enter\s*\(\s*(\d+)U?\s*\)', text))
                actual_exit = collections.Counter(int(x) for x in re.findall(r'Cp_Exit\s*\(\s*(\d+)U?\s*\)', text))
                want_enter = collections.Counter({s['id']: 1 for s in expected})
                want_exit = collections.Counter({s['id']: len(s['exit_lines']) for s in expected})
                raw = re.findall(r'\b(?:task(?:ENTER|EXIT)_CRITICAL|vPort(?:Enter|Exit)Critical)\s*\(\s*\)\s*;', text)
                if actual_enter != want_enter or actual_exit != want_exit or raw:
                    raise RuntimeError(f'Coverage mismatch {path}: {actual_enter}/{actual_exit}, raw={raw}')
                row = next(a for a in audit if a['file'] == path.relative_to(ROOT).as_posix())
                row.update(enter_calls=dict(actual_enter), exit_calls=dict(actual_exit), raw_bypass_calls=raw,
                           preprocessed_sha256=hash_file(pp), exact_once=True)
            run([str(BIN/'armcc.exe'), *options, '-c', str(path), '-o', str(obj)])
        objects.append(obj)
    scatter = project.findtext('.//TargetArmAds/LDads/ScatterFile') or 'sbus_memory.sct'
    run([str(BIN/'armlink.exe'), '--cpu', 'Cortex-M7.fp.dp', *map(str, objects), '--scatter', scatter,
         '--strict', '--map', '--symbols', '--xref', '--callgraph', '--list', str(out/'firmware.map'), '-o', str(out/'firmware.axf')])
    run([str(BIN/'fromelf.exe'), '--i32combined', '--output', str(out/'firmware.hex'), str(out/'firmware.axf')])
    after = {p.relative_to(ROOT).as_posix(): hash_file(p) for p in tracked}
    if before != after: raise RuntimeError('Production inputs changed')
    diagnostics = '\n'.join(transcript)
    result = dict(source_files=len(objects), application_sites=len(sites),
                  wrapped_entry_calls=len(sites), wrapped_exit_calls=sum(len(s['exit_lines']) for s in sites),
                  warnings=len(re.findall(r'warning:', diagnostics, re.I)), errors=len(re.findall(r'error:', diagnostics, re.I)),
                  production_inputs_unchanged=True, hex_sha256=hash_file(out/'firmware.hex'), runtime='NOT MEASURED')
    save(out/'preprocessor_audit.json', json.dumps(audit, indent=2))
    save(out/'build_result.json', json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__': main()
