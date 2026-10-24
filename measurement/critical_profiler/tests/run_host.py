from pathlib import Path
import json
import subprocess
import resource
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
root = Path(__file__).resolve().parents[3]
out = root/'audit/dwt_profiler/host'
out.mkdir(parents=True, exist_ok=True)
tests = Path(__file__).resolve().parent
exe = out/'test_profiler'
command = ['gcc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(tests),
           str(tests/'test_profiler.c'), '-o', str(exe)]
def call(args):
    p = subprocess.run(args, cwd=root, capture_output=True, timeout=30)
    return dict(command=args, exit_code=p.returncode, stdout=p.stdout.decode(errors='replace'),
                stderr=p.stderr.decode(errors='replace'))
results = {'compile': call(command)}
def save():
    (out/'results.json').write_bytes(json.dumps(results, indent=2).replace('\n', '\r\n').encode())
save()
assert results['compile']['exit_code'] == 0, results
for case in ('nested', 'wrap', 'preemption_envelope', 'post_exit_task', 'startup', 'unbalanced',
             'wrong_pair', 'depth_overflow', 'clock_faults', 'context_mask', 'counter_duration_site',
             'active_snapshot'):
    results[case] = call([str(exe), case])
    save()
    assert results[case]['exit_code'] == 0, results[case]
    print(case, 'PASS', flush=True)

generated=root/'audit/dwt_profiler/measurement_ac5/generated'
sites=json.loads((generated.parent/'coverage.json').read_text())
lines='static const HostSiteLine host_site_lines[] = {\n'
for s in sites:
    for line in [s['entry_line'],*s['exit_lines']]:
        lines+='    {"%s", %dU, %dU},\n' % (s['file'].split('/')[-1],line,s['id'])
(out/'host_site_lines.h').write_bytes((lines+'};\n').replace('\n','\r\n').encode())
integration=out/'test_heater_integration'
fixture=(root/'tests/pid_tuning/test_heater_characterization.c').read_text()
assert fixture.count('int main(void)')==1
(out/'heater_body.inc').write_bytes(fixture.replace('int main(void)','int heater_characterization_main(void)').replace('\n','\r\n').encode())
command=['gcc','-std=c11','-O2','-fno-fast-math','-ffp-contract=off',
         '-I',str(tests),'-I',str(tests.parent),'-I',str(out),'-I',str(generated),
         '-I',str(root/'tests/pid_tuning'),
         '-I',str(root/'tests/pid_tuning/stubs'),str(tests/'test_heater_integration.c'),
         '-lm','-o',str(integration)]
results['heater_compile']=call(command); save()
assert results['heater_compile']['exit_code']==0,results['heater_compile']
results['heater_integration']=call([str(integration)]); save()
assert results['heater_integration']['exit_code']==0,results['heater_integration']
print('unchanged heater/BMI nested integration PASS',flush=True)
tap=out/'test_telemetry_tap'
command=['gcc','-std=c11','-O2','-I',str(tests.parent),'-I',str(root/'App'),
         '-I',str(root/'Drivers/CMSIS/RTOS2/Include'),
         '-I',str(root/'tests/pid_tuning/stubs'),str(tests/'test_telemetry_tap.c'),'-lm','-o',str(tap)]
results['tap_compile']=call(command); save()
assert results['tap_compile']['exit_code']==0,results['tap_compile']
results['telemetry_tap']=call([str(tap)]); save()
assert results['telemetry_tap']['exit_code']==0,results['telemetry_tap']
print('telemetry frame/status preservation PASS',flush=True)
