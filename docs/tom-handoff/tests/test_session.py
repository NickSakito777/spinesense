"""Synthetic software checks only: no Windows, serial hardware or scientific QA claim."""
import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'tools' / 'session.py'
spec = importlib.util.spec_from_file_location('session', MODULE)
s = importlib.util.module_from_spec(spec); spec.loader.exec_module(s)


def sample(ms=100, sensor='IMU0'):
    return f'{ms}\t{sensor}\tlabel\t0x10\t0\t0\t1000\t0\t0\t0\t1\t0\t0\t0\n'.encode()


class FakeSerial:
    port = 'FAKE'; baudrate = 921600
    def __init__(self, chunks): self.chunks = iter(chunks)
    def readline(self):
        try: return next(self.chunks)
        except StopIteration: raise KeyboardInterrupt()


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.out = io.StringIO()
        self.redirect = contextlib.redirect_stdout(self.out); self.redirect.__enter__()
        self.folder = s.create(self.temp.name, 'CHECK', 0)
    def tearDown(self):
        self.redirect.__exit__(None,None,None); self.temp.cleanup()

    def run_capture(self, kind='imu', chunks=None):
        return s.capture_stream(FakeSerial(chunks or [sample()]), self.folder, kind, 20)

    def complete_session(self, missing=None, reset=False):
        for kind in ('tap','imu'):
            chunks = [sample(ms, sensor) for ms in (100, 50 if reset else 200) for sensor in sorted(s.SENSORS) if sensor != missing]
            ticks = iter([dict(host_unix_ns=10_000_000_000 + t*1000000, host_monotonic_ns=t*1000000) for t in range(0,1000,10)])
            with patch.object(s, 'stamp', side_effect=lambda:next(ticks)):
                self.run_capture(kind,chunks)
        # Capture valid timestamps 10..100; cues span 15..90.
        order = s.sequence(1,101)
        meta = {'status':'complete','machine':s.machine(),'seed':101,'reps':1,'order':order}
        s.save_json(self.folder/'cue_run.json',meta,True)
        events = [('pre_neutral',0,'')]
        for i,m in enumerate(order,1): events.extend((e,i,m) for e in ('next','go','return_neutral'))
        events.extend([('post_neutral',0,''),('complete',0,'')])
        with (self.folder/'cues.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=s.CUE_FIELDS); w.writeheader()
            for i,(e,b,m) in enumerate(events):
                t=51_000_000+i*100000
                w.writerow(dict(event=e,bout=b,movement=m,host_monotonic_ns=t,host_unix_ns=10_000_000_000+t))
        with (self.folder/'placement.csv').open('w',newline='') as f:
            w=csv.writer(f); w.writerow(['sensor','body_position']); w.writerows(zip(sorted(s.SENSORS),s.POSITIONS))
        (self.folder/'photos'/'placeholder.jpg').write_bytes(b'synthetic; checker only checks presence')

    def last_report(self):
        return s.read_json(sorted(self.folder.glob('qc_*.json'),key=lambda p:p.stat().st_mtime_ns)[-1])

    def test_byte_preservation_partial_lines_and_sidecar(self):
        raw = sample() + b'bad\xffline\ntrailing'
        self.run_capture(chunks=[raw[:5], raw[5:14], raw[14:]])
        self.assertEqual((self.folder/'imu.log').read_bytes(),raw)
        rows=s.read_csv(self.folder/'imu_timing.csv')
        self.assertEqual(len(rows),3)
        self.assertEqual(rows[-1]['complete'],'0')
        self.assertEqual(sum(int(r['bytes']) for r in rows),len(raw))
        self.assertEqual(rows[0]['valid_sample'],'1')

    def test_collision_does_not_overwrite(self):
        self.run_capture(); before=(self.folder/'imu.log').read_bytes()
        with self.assertRaises(ValueError): self.run_capture(chunks=[b'different\n'])
        self.assertEqual((self.folder/'imu.log').read_bytes(),before)

    def test_serial_error_keeps_partial_and_records_error(self):
        class Broken(FakeSerial):
            def readline(self): raise OSError('disconnected')
        self.assertEqual(s.capture_stream(Broken([]),self.folder,'imu',20),2)
        self.assertEqual(s.read_json(self.folder/'imu_capture.json')['status'],'error')

    def test_reset_segments(self):
        self.run_capture(chunks=[sample(100),sample(101),sample(2)])
        self.assertEqual([r['segment'] for r in s.read_csv(self.folder/'imu_timing.csv')],['0','0','1'])

    def test_complete_is_manual_review_never_pass(self):
        self.complete_session(); self.assertEqual(s.check(self.folder),1)
        self.assertEqual(self.last_report()['status'],'REVIEW_REQUIRED')

    def test_missing_sensor_blocks(self):
        self.complete_session(missing='IMU4'); self.assertEqual(s.check(self.folder),2)
        self.assertTrue(any('missing sensors' in e for e in self.last_report()['errors']))

    def test_reset_blocks(self):
        self.complete_session(reset=True); self.assertEqual(s.check(self.folder),2)
        self.assertTrue(any('clock reset' in e for e in self.last_report()['errors']))

    def test_missing_cues_blocks(self):
        self.complete_session(); (self.folder/'cues.csv').unlink(); self.assertEqual(s.check(self.folder),2)

    def test_missing_timing_blocks(self):
        self.complete_session(); (self.folder/'imu_timing.csv').unlink(); self.assertEqual(s.check(self.folder),2)

    def test_raw_sidecar_tampering_blocks(self):
        self.complete_session()
        with (self.folder/'imu.log').open('ab') as f:f.write(b'EXTRA\n')
        self.assertEqual(s.check(self.folder),2)

    def test_incomplete_cue_sequence_blocks(self):
        self.complete_session(); p=self.folder/'cues.csv'
        p.write_text('\n'.join(p.read_text().splitlines()[:-1])+'\n')
        self.assertEqual(s.check(self.folder),2)

    def test_same_pc_and_wall_clock_checked(self):
        self.complete_session(); p=self.folder/'cue_run.json'; data=s.read_json(p)
        data['machine']['hostname']='another-computer'; s.save_json(p,data)
        self.assertEqual(s.check(self.folder),2)

    def rewrite_csv(self, path, rows):
        with path.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)

    def test_wall_clock_jump_blocks(self):
        self.complete_session(); p=self.folder/'imu_timing.csv'; rows=s.read_csv(p)
        rows[-1]['host_unix_ns']=str(int(rows[-1]['host_unix_ns'])+500_000_000)
        self.rewrite_csv(p,rows); self.assertEqual(s.check(self.folder),2)
        self.assertTrue(any('clock' in e for e in self.last_report()['errors']))

    def test_segment_label_tampering_blocks(self):
        self.complete_session(); p=self.folder/'imu_timing.csv'; rows=s.read_csv(p)
        rows[-1]['segment']='4'; self.rewrite_csv(p,rows)
        self.assertEqual(s.check(self.folder),2)
        self.assertTrue(any('segment label' in e for e in self.last_report()['errors']))

    def test_large_receive_gap_blocks(self):
        self.complete_session(); p=self.folder/'imu_timing.csv'; rows=s.read_csv(p)
        for row in rows[5:]:
            for k in ('host_unix_ns','host_monotonic_ns'): row[k]=str(int(row[k])+2_000_000_000)
        self.rewrite_csv(p,rows); self.assertEqual(s.check(self.folder),2)
        self.assertTrue(any('gap exceeds' in e for e in self.last_report()['errors']))

    def prepare_running_capture(self):
        self.run_capture(); p=self.folder/'imu_capture.json'; meta=s.read_json(p)
        meta['status']='recording'; s.save_json(p,meta)

    def test_cues_complete_writes_every_event(self):
        self.prepare_running_capture()
        with patch.object(s.time,'sleep'), patch.object(s,'beep'):
            self.assertEqual(s.cues(self.folder,1,101),0)
        rows=s.read_csv(self.folder/'cues.csv')
        self.assertEqual(len(rows),21)
        self.assertEqual(rows[0]['event'],'pre_neutral')
        self.assertEqual(rows[-1]['event'],'complete')
        with self.assertRaises(ValueError): s.cues(self.folder,1,101)

    def test_cue_interrupt_retains_event(self):
        self.prepare_running_capture()
        with patch.object(s.time,'sleep',side_effect=KeyboardInterrupt), patch.object(s,'beep'):
            self.assertEqual(s.cues(self.folder,1,101),2)
        self.assertEqual(s.read_json(self.folder/'cue_run.json')['status'],'interrupted')
        self.assertEqual(len(s.read_csv(self.folder/'cues.csv')),1)

    def test_seeded_sequence_balanced_no_triples(self):
        seq=s.sequence(10,101)
        self.assertEqual(seq,s.sequence(10,101)); self.assertEqual(len(seq),60)
        self.assertTrue(all(seq.count(m)==10 for m in s.MOVEMENTS))
        self.assertTrue(all(len(set(seq[i:i+3]))>1 for i in range(len(seq)-2)))

    def test_nonfinite_samples_rejected(self):
        self.assertFalse(s.sample_info(sample().replace(b'1000',b'nan'))[2])

if __name__=='__main__': unittest.main()
