"""Offline-only preparation/decoding. Never opens a probe or executes device commands."""
from pathlib import Path
import argparse
import csv
import hashlib
import io
import json
import re
import struct
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
HEADER = ('version sequence complete valid faults core_clock_hz site_count current_depth max_depth '
          'startup_pairs outer_hits max_outer_body_cycles max_outer_envelope_cycles '
          'max_outer_body_us_ceil max_outer_envelope_us_ceil max_accounting_body_cycles '
          'max_snapshot_envelope_cycles measurement_rc_stack_bytes catalog').split()
SITE = ('hits max_body_cycles max_envelope_cycles max_enter_envelope_cycles max_exit_envelope_cycles '
        'max_enter_bookkeeping_cycles max_exit_bookkeeping_cycles max_inclusive_depth '
        'max_body_us_ceil max_envelope_us_ceil').split()
FAULTS = {1:'DEPTH',2:'PAIR',4:'CLOCK',8:'CONTEXT',16:'EXTERNAL_MASK',32:'DURATION',
          64:'COUNTER',128:'SITE',256:'ACTIVE_SNAPSHOT'}
TASKS = [
    ('defaultTask','defaultTaskHandle',1024,1024), ('RcTask','rcTaskHandle',512,1024),
    ('OpticalFlowTask','opticalFlowTaskHandle',1024,1024), ('Bmi088Task','bmi088TaskHandle',1024,1024),
    ('BlackboxTask','blackboxTaskHandle',1024,1024), ('BlackboxStorage','blackboxStorageTaskHandle',4096,4096),
    ('FlightControlTask','flightControlTaskHandle',1024,1024), ('ws2812','ws2812TaskHandle',1024,1024),
    ('IDLE','xIdleTaskHandle',512,512), ('Tmr Svc','xTimerTaskHandle',1024,1024)]


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(text.replace('\r\n','\n').replace('\n','\r\n').encode('utf-8'))


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


class Elf32:
    def __init__(self, path):
        self.data=path.read_bytes()
        if self.data[:6]!=b'\x7fELF\x01\x01': raise ValueError('Expected ELF32 little-endian AXF/object')
        self.kind=struct.unpack_from('<H',self.data,16)[0]
        off=struct.unpack_from('<I',self.data,32)[0]
        size,count,_=struct.unpack_from('<HHH',self.data,46)
        self.sections=[struct.unpack_from('<10I',self.data,off+i*size) for i in range(count)]
        self.symbols={}
        for h in self.sections:
            if h[1]!=2: continue
            sh=self.sections[h[6]];strings=self.data[sh[4]:sh[4]+sh[5]]
            for pos in range(h[4],h[4]+h[5],h[9]):
                name,value,length,info,other,section=struct.unpack_from('<IIIBBH',self.data,pos)
                name=strings[name:].split(b'\0',1)[0].decode('utf-8','replace')
                if name: self.symbols[name]=(value,length,section)
    def value_bytes(self,name):
        value,length,index=self.symbols[name];h=self.sections[index]
        offset=h[4]+value-(0 if self.kind==1 else h[3])
        return self.data[offset:offset+length]


def map_symbol(text, name, owner=None):
    rows=[]
    for line in text.splitlines():
        m=re.match(r'\s*'+re.escape(name)+r'\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)\s+(.+)',line)
        if m and (owner is None or owner in m[3]): rows.append((int(m[1],16),int(m[2])))
    if len(rows)!=1: raise ValueError(f'Expected one map symbol {name}: {rows}')
    return rows[0]


def prepare(build, out):
    build=build.resolve();out.mkdir(parents=True,exist_ok=True)
    elf=Elf32(build/'firmware.axf');text=(build/'firmware.map').read_text(encoding='utf-8',errors='replace')
    sites=json.loads((build/'coverage.json').read_text(encoding='utf-8'))
    addr,size=map_symbol(text,'g_cp_snapshot','critical_profiler.o')
    if elf.symbols['g_cp_snapshot'][:2]!=(addr,size): raise ValueError('map/AXF snapshot mismatch')
    # Compile-only probe: derives every offset/size from the actual AC5 ABI and
    # actual private FreeRTOS TCB definition. This object is NEVER linked/flashed.
    fields=['sizeof(CpSnapshot)','sizeof(CpSiteStats)','offsetof(CpSnapshot, sites)',
            'sizeof(void *)','sizeof(StackType_t)','offsetof(TCB_t, pxStack)','CP_SITE_COUNT']
    fields += [f'offsetof(CpSnapshot, {n})' for n in HEADER]
    fields += [f'offsetof(CpSiteStats, {n})' for n in SITE]
    fields += ['tskSTACK_FILL_BYTE','(uint32_t)portSTACK_GROWTH']
    probe='#include "critical_profiler.h"\n#include <stddef.h>\n#include "tasks.c"\n'
    probe+='const uint32_t g_export_layout[] = {\n'+',\n'.join(fields)+'\n};\n'
    write(out/'layout_probe.c',probe)
    project=ET.parse(ROOT/'MDK-ARM/CtrlBoard-H7.uvprojx')
    ctl=project.find('.//TargetArmAds/Cads/VariousControls')
    args=[r'D:\keil5\ARM\ARMCC\Bin\armcc.exe','--cpu=Cortex-M7.fp.dp','--c99','--gnu','-O3','-g','-c']
    for inc in ctl.findtext('IncludePath').split(';'): args += ['-I',inc.strip()]
    for define in ctl.findtext('Define').split(','):args+=['-D',define.strip()]
    for p in (ROOT/'measurement/critical_profiler',build/'generated',ROOT/'Middlewares/Third_Party/FreeRTOS/Source'):
        args+=['-I',str(p)]
    args += [str((out/'layout_probe.c').resolve()),'-o',str((out/'layout_probe.o').resolve())]
    p=subprocess.run(args,cwd=ROOT/'MDK-ARM',capture_output=True,timeout=120)
    write(out/'layout_probe.log',p.stdout.decode(errors='replace')+p.stderr.decode(errors='replace'))
    if p.returncode:raise RuntimeError('Layout probe compilation failed')
    raw=Elf32(out/'layout_probe.o').value_bytes('g_export_layout')
    values=struct.unpack('<'+'I'*(len(raw)//4),raw)
    expected=[76+40*len(sites),40,76,4,4]
    if list(values[:5])!=expected or values[6]!=len(sites) or values[0]!=size:
        raise ValueError('Unsupported layout; do not decode with assumed offsets')
    if list(values[7:7+len(HEADER)])!=list(range(0,76,4)) or list(values[7+len(HEADER):-2])!=list(range(0,40,4)):
        raise ValueError('Offset mismatch')
    if values[-2:]!=(0xA5,0xffffffff): raise ValueError('Unsupported stack fill/direction')
    budgets={}
    for line in (ROOT/'measurement/critical_profiler/COVERAGE_MATRIX.txt').read_text(encoding='utf-8').splitlines():
        columns=[x.strip() for x in line.split('|')]
        if len(columns)==7 and columns[0].isdigit():budgets[int(columns[0])]=float(columns[5])
    for s in sites:s['budget_us']=budgets[s['id']]
    symbols={}
    for name in ('g_cp_sites','s_faults','s_depth','xFreeBytesRemaining','xMinimumEverFreeBytesRemaining'):
        symbols[name]=map_symbol(text,name,'critical_profiler.o' if name in ('s_faults','s_depth') else None)[0]
    tasks=[]
    for name,handle,production,measurement in TASKS:
        tasks.append(dict(name=name,handle_symbol=handle,handle_address=map_symbol(text,handle)[0],
                          production_stack_bytes=production,measurement_stack_bytes=measurement))
    meta=dict(format_version=1,axf_sha256=digest(build/'firmware.axf'),map_sha256=digest(build/'firmware.map'),
              hex_sha256=digest(build/'firmware.hex'),snapshot_address=addr,snapshot_size=size,header_bytes=76,
              site_bytes=40,site_count=len(sites),catalog_address=symbols['g_cp_sites'],
              pointer_bytes=4,stack_word_bytes=4,stack_fill_byte=values[-2],stack_growth=-1,
              tcb_pxStack_offset=values[5],symbols=symbols,sites=sites,tasks=tasks)
    write(out/'manifest.json',json.dumps(meta,indent=2))
    dumps=[('header_before.bin',addr,76),('snapshot.bin',addr,size),('header_after.bin',addr,76),
           ('live_faults.bin',symbols['s_faults'],4),('live_depth.bin',symbols['s_depth'],4),
           ('heap_min.bin',symbols['xMinimumEverFreeBytesRemaining'],4),('heap_free.bin',symbols['xFreeBytesRemaining'],4)]
    dumps += [(t['handle_symbol']+'.handle.bin',t['handle_address'],4) for t in tasks]
    # Operator executes ONLY after ending the scenario. No reset/resume/load.
    write(out/'jlink_dump.jlink','halt\n'+''.join(f'SaveBin {n} 0x{a:08X} 0x{z:X}\n' for n,a,z in dumps))
    ocd='halt\nwait_halt 2000\n'+''.join(f'dump_image {n} 0x{a:08X} 0x{z:X}\n' for n,a,z in dumps)
    ocd+='\n# Existing task stack blocks, read only after final halt.\n'
    for t in tasks:
        key=t['handle_symbol'];n=t['measurement_stack_bytes']
        ocd+=f'set tcb [lindex [read_memory 0x{t["handle_address"]:08X} 32 1] 0]\n'
        ocd+=f'if {{$tcb < 0x20000000 || $tcb + {values[5]+4} > 0x20020000}} {{ error "Invalid task handle: {key}" }}\n'
        ocd+=f'set base [lindex [read_memory [expr {{$tcb + {values[5]}}}] 32 1] 0]\n'
        ocd+=f'if {{$base < 0x20000000 || $base + {n} > 0x20020000}} {{ error "Invalid stack range: {key}" }}\n'
        ocd+=f'dump_image {key}.stack.bin $base {n}\n'
    write(out/'openocd_dump.tcl',ocd)
    # J-Link has no Tcl pointer arithmetic: capture bounded TCB prefixes first,
    # then generate stack dump commands offline from those files.
    write(out/'task_addresses.txt','\n'.join(f'{t["name"]}: {t["handle_symbol"]} @ 0x{t["handle_address"]:08X}; stack {t["measurement_stack_bytes"]} B' for t in tasks)+'\n')
    print(json.dumps({k:meta[k] for k in ('snapshot_address','snapshot_size','site_count','tcb_pxStack_offset','axf_sha256')},indent=2))


def stack_commands(meta, capture, phase):
    rows=[]
    for t in meta['tasks']:
        key=t['handle_symbol']
        if phase=='tcb':
            data=(capture/(key+'.handle.bin')).read_bytes()
            if len(data)!=4:raise ValueError('Bad handle dump')
            address=struct.unpack('<I',data)[0];size=meta['tcb_pxStack_offset']+4
            filename=key+'.tcb.bin'
        else:
            data=(capture/(key+'.tcb.bin')).read_bytes()
            address=struct.unpack_from('<I',data,meta['tcb_pxStack_offset'])[0]
            size=t['measurement_stack_bytes'];filename=key+'.stack.bin'
        if not 0x20000000<=address<address+size<=0x20020000:raise ValueError(f'Unexpected DTCM range {key}: {address:#x}+{size}')
        rows.append(f'SaveBin {filename} 0x{address:08X} 0x{size:X}')
    write(capture/('jlink_'+phase+'.jlink'),'\n'.join(rows)+'\n')


def decode(meta, capture):
    data=(capture/'snapshot.bin').read_bytes()
    if len(data)!=meta['snapshot_size']:raise ValueError('Snapshot must be complete, exact length')
    values=struct.unpack_from('<19I',data)
    header=dict(zip(HEADER,values));reasons=[]
    for n in ('header_before.bin','header_after.bin'):
        if (capture/n).read_bytes()!=data[:76]:raise ValueError('Torn/changing snapshot header')
    if header['version']!=1 or header['site_count']!=meta['site_count'] or header['catalog']!=meta['catalog_address']:
        raise ValueError('Build/layout/catalog mismatch')
    clock=header['core_clock_hz']
    if clock<1000000 or clock%1000000:raise ValueError('Unsupported/invalid recorded clock')
    if header['complete']!=1:reasons.append('INCOMPLETE')
    if header['valid']!=1:reasons.append('VALID_FLAG_FALSE')
    if header['faults']:reasons.append('FAULTS')
    if header['current_depth'] or header['max_depth']>16:reasons.append('DEPTH')
    if not header['sequence'] or not header['outer_hits']:reasons.append('NO_COMPLETE_RUNTIME_CAPTURE')
    if header['measurement_rc_stack_bytes']!=1024:reasons.append('UNEXPECTED_MEASUREMENT_STACK')
    def word(name):
        b=(capture/name).read_bytes()
        if len(b)!=4:raise ValueError('Bad word dump: '+name)
        return struct.unpack('<I',b)[0]
    live_faults,live_depth=word('live_faults.bin'),word('live_depth.bin')
    if live_faults or live_depth:reasons.append('LIVE_GUARD_REJECTED')
    divisor=clock//1000000
    def ceil_us(v):return v//divisor+bool(v%divisor)
    for body,us in [('max_outer_body_cycles','max_outer_body_us_ceil'),('max_outer_envelope_cycles','max_outer_envelope_us_ceil')]:
        if header[us]!=ceil_us(header[body]):raise ValueError('Corrupt cycles/us relation')
    if header['max_outer_envelope_cycles']<header['max_outer_body_cycles']:raise ValueError('Invalid outer envelope')
    rows=[]
    for i,site in enumerate(meta['sites']):
        v=dict(zip(SITE,struct.unpack_from('<10I',data,76+40*i)))
        for cy,us in [('max_body_cycles','max_body_us_ceil'),('max_envelope_cycles','max_envelope_us_ceil')]:
            if v[us]!=ceil_us(v[cy]):raise ValueError('Corrupt site cycles/us relation')
        if v['max_envelope_cycles']<v['max_body_cycles'] or v['max_inclusive_depth']>16:
            raise ValueError('Invalid site interval/depth')
        if v['hits'] and (not v['max_body_cycles'] or not v['max_inclusive_depth']):
            raise ValueError('Hit exists without a completed timing sample; do not report zero duration')
        status='NOT MEASURED' if not v['hits'] else ('REJECTED RUN' if reasons else 'OBSERVED')
        row=dict(ID=site['id'],file=site['file'],function=site['function'],line=site['entry_line'],
                 status=status,budget_us=site['budget_us'],**v)
        for metric in ('body','envelope'):
            value=v['max_'+metric+'_us_ceil'];budget=site['budget_us']
            row[metric+'_margin_percent']=round((budget-value)/budget*100,3) if v['hits'] and not reasons else None
        rows.append(row)
    task_rows=[]
    for t in meta['tasks']:
        p=capture/(t['handle_symbol']+'.stack.bin');row=dict(t)
        row.update(status='NOT MEASURED',minimum_free_words=None,minimum_free_bytes=None)
        if p.exists():
            b=p.read_bytes()
            if len(b)!=t['measurement_stack_bytes']:raise ValueError('Wrong stack dump size')
            untouched=0
            for byte in b:
                if byte!=0xA5:break
                untouched+=1
            row.update(status='OBSERVED_FILL_PATTERN',minimum_free_words=untouched//4,minimum_free_bytes=(untouched//4)*4)
            if untouched==len(b):
                row.update(status='REVIEW_ALL_FILL_NO_SENTINEL',minimum_free_words=None,minimum_free_bytes=None)
        row['stage5_followup']=(t['name']=='RcTask' and row['minimum_free_bytes'] is not None and row['minimum_free_bytes']<128)
        task_rows.append(row)
    report=dict(snapshot=header,run_status='REJECTED' if reasons else 'INTERNALLY_CONSISTENT_REQUIRES_RUN_LOG',
                rejection_reasons=reasons,live_faults=live_faults,live_depth=live_depth,
                faults_decoded=[name for bit,name in FAULTS.items() if (header['faults']|live_faults)&bit],
                unknown_fault_bits=(header['faults']|live_faults)&~511,
                accounting_body_us_ceil=ceil_us(header['max_accounting_body_cycles']),
                snapshot_envelope_us_ceil=ceil_us(header['max_snapshot_envelope_cycles']),
                heap_min_bytes=word('heap_min.bin'),heap_free_bytes=word('heap_free.bin'),
                snapshot_sha256=digest(capture/'snapshot.bin'),expected_axf_sha256=meta['axf_sha256'],
                stage5_todos=[f"RcTask {t['minimum_free_bytes']} B minimum free (<128 B), measurement allocation {t['measurement_stack_bytes']} B; inspect production separately" for t in task_rows if t['stage5_followup']],
                sites=rows,tasks=task_rows)
    write(capture/'report.json',json.dumps(report,indent=2))
    for name,table in [('sites.csv',rows),('tasks.csv',task_rows)]:
        b=io.StringIO();w=csv.DictWriter(b,fieldnames=list(table[0]));w.writeheader();w.writerows(table)
        write(capture/name,b.getvalue())
    print(json.dumps({k:v for k,v in report.items() if k not in ('sites','tasks')},indent=2))
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__);sub=ap.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare');p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    for name in ('decode','jlink-tcb','jlink-stacks'):
        p=sub.add_parser(name);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--capture',type=Path,required=True)
    a=ap.parse_args()
    if a.command=='prepare':prepare(a.build,a.out)
    else:
        meta=json.loads(a.manifest.read_text(encoding='utf-8'))
        if a.command=='decode':decode(meta,a.capture)
        else:stack_commands(meta,a.capture,'tcb' if a.command=='jlink-tcb' else 'stacks')


if __name__=='__main__':main()
