"""Insert one scheduling observation into the actual ESC translation unit.

No control/safety logic is copied into the test. Removing the single observation
restores the current source bytes exactly. The RTOS model decides when a pending
timeout task is runnable, including deferral across nested critical sections.
"""
from pathlib import Path
import argparse,hashlib,json
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('output',type=Path)
parser.add_argument('--source',type=Path,help='Explicit frozen pre-fix source for differential replay')
args=parser.parse_args()
root=Path(__file__).resolve().parents[2]
source=args.source if args.source else root/'App/esc_pwm.c'
data=source.read_bytes()
needle=b'        g_esc_pwm_state.armed = 1U;'
assert data.count(needle)==1,'Expected exactly one arming commit'
newline=b'\r\n' if b'\r\n' in data else b'\n'
hook=b'        Test_ArmingCommitPoint(); /* HOST SCHEDULING OBSERVATION ONLY */'+newline
derived=data.replace(needle,hook+needle,1)
assert derived.replace(hook,b'',1)==data
args.output.mkdir(parents=True,exist_ok=True)
(args.output/'esc_arming_actual.inc').write_bytes(derived)
(args.output/'esc_arming_source.json').write_text(json.dumps({
    'source':'App/esc_pwm.c','sha256':hashlib.sha256(data).hexdigest(),
    'single_observation_removed_restores_exact_source':True,
    'logic_changes':[]},indent=2),encoding='utf-8')
