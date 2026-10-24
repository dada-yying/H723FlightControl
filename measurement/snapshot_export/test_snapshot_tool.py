"""Synthetic input only: no hardware data and no probe access."""
from pathlib import Path
import contextlib
import io
import json
import struct
import unittest
import snapshot_tool as tool

ROOT=Path(__file__).resolve().parents[2]
META=json.loads((ROOT/'audit/dwt_export/bundle/manifest.json').read_text(encoding='utf-8'))
OUT=ROOT/'audit/dwt_export/SYNTHETIC_TESTS'


class DecodeTests(unittest.TestCase):
    def setUp(self):
        self.path=OUT/self._testMethodName;self.path.mkdir(parents=True,exist_ok=True)
        h={n:0 for n in tool.HEADER}
        h.update(version=1,sequence=100,complete=1,valid=1,core_clock_hz=480000000,
                 site_count=META['site_count'],max_depth=2,startup_pairs=3,outer_hits=1,
                 max_outer_body_cycles=480,max_outer_envelope_cycles=960,
                 max_outer_body_us_ceil=1,max_outer_envelope_us_ceil=2,
                 max_accounting_body_cycles=24,max_snapshot_envelope_cycles=1440,
                 measurement_rc_stack_bytes=1024,catalog=META['catalog_address'])
        self.h=h
        self.rows=[{n:0 for n in tool.SITE} for _ in META['sites']]
        self.site=next(s['id'] for s in META['sites'] if s['file']=='App/esc_pwm.c' and s['budget_us']==2)
        self.rows[self.site].update(hits=10,max_body_cycles=480,max_envelope_cycles=960,
                                    max_inclusive_depth=2,max_body_us_ceil=1,max_envelope_us_ceil=2)
        for n,v in [('live_faults.bin',0),('live_depth.bin',0),('heap_min.bin',1024),('heap_free.bin',1400)]:
            (self.path/n).write_bytes(struct.pack('<I',v))
        self.emit()

    def emit(self):
        b=struct.pack('<19I',*(self.h[n] for n in tool.HEADER))
        for row in self.rows:b+=struct.pack('<10I',*(row[n] for n in tool.SITE))
        (self.path/'snapshot.bin').write_bytes(b)
        for n in ('header_before.bin','header_after.bin'):(self.path/n).write_bytes(b[:76])

    def decode(self):
        with contextlib.redirect_stdout(io.StringIO()):return tool.decode(META,self.path)

    def test_complete_fields_and_margins(self):
        r=self.decode();s=r['sites'][self.site]
        self.assertEqual(s['body_margin_percent'],50)
        self.assertEqual(s['envelope_margin_percent'],0)
        self.assertEqual(r['snapshot']['startup_pairs'],3)
        self.assertEqual(r['snapshot_envelope_us_ceil'],3)
        self.assertEqual(r['heap_min_bytes'],1024)
        self.assertEqual(r['sites'][0]['status'],'NOT MEASURED')

    def test_torn_header(self):
        b=bytearray((self.path/'header_after.bin').read_bytes());b[4]^=1
        (self.path/'header_after.bin').write_bytes(b)
        with self.assertRaises(ValueError):self.decode()

    def test_short_dump(self):
        p=self.path/'snapshot.bin';p.write_bytes(p.read_bytes()[:-1])
        with self.assertRaises(ValueError):self.decode()

    def test_build_mismatch(self):
        self.h['catalog']+=4;self.emit()
        with self.assertRaises(ValueError):self.decode()

    def test_fault_rejects_run(self):
        self.h['faults']=3;self.emit();r=self.decode()
        self.assertEqual(r['run_status'],'REJECTED')
        self.assertEqual(r['sites'][self.site]['status'],'REJECTED RUN')
        self.assertIsNone(r['sites'][self.site]['body_margin_percent'])

    def test_live_depth_rejects_run(self):
        (self.path/'live_depth.bin').write_bytes(struct.pack('<I',1))
        self.assertEqual(self.decode()['run_status'],'REJECTED')

    def test_us_integrity(self):
        self.rows[self.site]['max_body_us_ceil']=2;self.emit()
        with self.assertRaises(ValueError):self.decode()

    def test_zero_hit_not_zero_time_claim(self):
        r=self.decode();self.assertIsNone(r['sites'][0]['body_margin_percent'])
        self.assertEqual(r['sites'][0]['status'],'NOT MEASURED')

    def test_stack_scan_and_stage5_threshold(self):
        for t in META['tasks']:
            (self.path/(t['handle_symbol']+'.stack.bin')).write_bytes(b'\xa5'*64+b'\x00'*(t['measurement_stack_bytes']-64))
        r=self.decode();self.assertEqual(len(r['tasks']),10)
        self.assertTrue(all(t['minimum_free_words']==16 for t in r['tasks']))
        self.assertEqual(len(r['stage5_todos']),1)

    def test_jlink_pointer_dereference(self):
        for i,t in enumerate(META['tasks']):
            key=t['handle_symbol'];(self.path/(key+'.handle.bin')).write_bytes(struct.pack('<I',0x20001000+i*128))
            b=bytearray(META['tcb_pxStack_offset']+4)
            struct.pack_into('<I',b,META['tcb_pxStack_offset'],0x20005000+i*0x1000)
            (self.path/(key+'.tcb.bin')).write_bytes(b)
        tool.stack_commands(META,self.path,'tcb');tool.stack_commands(META,self.path,'stacks')
        self.assertEqual((self.path/'jlink_stacks.jlink').read_text().count('SaveBin'),10)

    def test_all_fill_stack_is_not_accepted_as_free(self):
        t=META['tasks'][1]
        (self.path/(t['handle_symbol']+'.stack.bin')).write_bytes(b'\xa5'*t['measurement_stack_bytes'])
        row=self.decode()['tasks'][1]
        self.assertEqual(row['status'],'REVIEW_ALL_FILL_NO_SENTINEL')
        self.assertIsNone(row['minimum_free_bytes'])

    def test_pending_accounting_not_zero_duration(self):
        self.rows[self.site]['max_body_cycles']=0
        self.rows[self.site]['max_body_us_ceil']=0
        self.emit()
        with self.assertRaises(ValueError):self.decode()


if __name__=='__main__':
    stream=io.StringIO()
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(DecodeTests)
    result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    text=stream.getvalue()
    (ROOT/'audit/dwt_export/parser_tests.txt').write_bytes(text.replace('\n','\r\n').encode())
    print(text,end='')
    raise SystemExit(0 if result.wasSuccessful() else 1)
