"""Independently recheck retained AC5 output, including missing/duplicate negatives."""
from pathlib import Path
import collections
import json
import re
import hashlib
import sys
from build_measurement import scrub
root=Path(__file__).resolve().parents[2]
out=(root/(sys.argv[1] if len(sys.argv)>1 else 'audit/dwt_profiler/measurement_ac5')).resolve()
sites=json.loads((out/'coverage.json').read_text())
records=json.loads((out/'preprocessor_audit.json').read_text())
inventory=[]
for folder in ('App','BSP','Core','Devices'):
    for p in (root/folder).rglob('*'):
        if p.suffix not in ('.c','.h'): continue
        text=scrub(p.read_text(encoding='utf-8'))
        for m in re.finditer(r'\btask(ENTER|EXIT)_CRITICAL\s*\(\s*\)\s*;',text):
            inventory.append((p.relative_to(root).as_posix(),m[1],text[:m.start()].count('\n')+1))
expected_inventory=[]
for r in records:
    expected_inventory.extend((r['file'],c['kind'],c['line']) for c in r['expected_calls'])
assert sorted(inventory)==sorted(expected_inventory),'Owned source/header inventory mismatch'

def verify(text, expected):
    enter=collections.Counter(int(n) for n in re.findall(r'Cp_Enter\s*\(\s*(\d+)U?\s*\)',text))
    leave=collections.Counter(int(n) for n in re.findall(r'Cp_Exit\s*\(\s*(\d+)U?\s*\)',text))
    raw=re.findall(r'\b(?:task(?:ENTER|EXIT)_CRITICAL|vPort(?:Enter|Exit)Critical)\s*\(\s*\)\s*;',text)
    return enter==collections.Counter({s['id']:1 for s in expected}) and leave==collections.Counter({s['id']:len(s['exit_lines']) for s in expected}) and not raw

for record in records:
    pp=out/'preprocessed'/(Path(record['file']).stem+'.i')
    text=pp.read_text(encoding='utf-8',errors='replace')
    expected=[s for s in sites if s['file']==record['file']]
    assert hashlib.sha256(pp.read_bytes()).hexdigest()==record['preprocessed_sha256']
    assert verify(text,expected),record['file']
    if expected:
        match=re.search(r'Cp_Enter\s*\(\s*\d+U?\s*\)',text)
        assert not verify(text[:match.start()]+text[match.end():],expected)
        assert not verify(text[:match.end()]+';'+match[0]+text[match.end():],expected)
    if record['file']=='Core/Src/freertos.c':
        assert len(re.findall(r'Cp_TelemetryTap\s*\(\s*frame\s*\)',text))==1
        assert len(re.findall(r'Cp_ThreadNew\s*\(\s*(?:Start\w+|WS2812_ColorTask)\s*,',text))==8
    print(record['file'],'EXACTLY ONCE',flush=True)

commands=json.loads((out/'commands.json').read_text())
for cmd in commands:
    if '--preinclude' in cmd:
        source=next((v for v in cmd if v.endswith('.c')),None)
        assert source and Path(source).relative_to(root).parts[0] in ('App','BSP','Core','Devices')
        assert 'critical_profiler.c' not in source and 'telemetry_adapter.c' not in source
print('PASS:',len(sites),'entries;',sum(len(s['exit_lines']) for s in sites),'exit call sites')
print('PASS negative controls: removed/duplicated wrapper calls rejected')
print('Kernel/HAL/internal BASEPRI/PRIMASK and profiler accounting: NOT APPLICATION COVERAGE')
