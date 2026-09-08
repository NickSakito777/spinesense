#!/usr/bin/env python3
"""Independent Southampton acquisition. Host receipt times are NOT sample times.

Requires Python >=3.10; pyserial is needed only for ports/capture.
All raw streams are kept byte-for-byte. Never reconnect or overwrite a recording.
"""
from __future__ import annotations
import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import random
import re
import socket
import sys
import time
import uuid
from collections import Counter, defaultdict

MOVEMENTS = ['flexion', 'extension', 'left bend', 'right bend', 'left twist', 'right twist']
POSITIONS = ['sacrum', 'lower back', 'mid back', 'upper back', 'sternum']
SENSORS = {f'IMU{i}' for i in range(5)}
TIMING_FIELDS = ['line', 'offset', 'bytes', 'complete', 'host_unix_ns', 'host_monotonic_ns', 'segment', 'mcu_ms', 'sensor', 'valid_sample']
CUE_FIELDS = ['event', 'bout', 'movement', 'host_unix_ns', 'host_monotonic_ns']


def stamp():
    return {'host_unix_ns': time.time_ns(), 'host_monotonic_ns': time.monotonic_ns()}


def machine():
    return {'hostname': socket.gethostname(), 'system': platform.system(), 'node': str(uuid.getnode())}


def save_json(path, value, exclusive=False):
    with Path(path).open('x' if exclusive else 'w', encoding='utf-8') as f:
        json.dump(value, f, indent=2)
        f.write('\n')


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def create(root, subject, round_no):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,32}', subject):
        raise ValueError('Subject must be a pseudonymous ID: 1–32 letters, digits, _ or -.')
    name = f'{subject}_r{round_no}_{dt.datetime.now(dt.timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:8]}'
    folder = Path(root) / name
    folder.mkdir(parents=True, exist_ok=False)
    save_json(folder / 'session.json', {'schema': 1, 'subject': subject, 'round': round_no, 'created': stamp(), 'machine': machine()}, True)
    with (folder / 'placement.csv').open('x', newline='', encoding='utf-8') as f:
        w = csv.writer(f); w.writerow(['sensor', 'body_position'])
        w.writerows((f'IMU{i}', '') for i in range(5))
    (folder / 'notes.txt').write_text('Operator:\nDate:\nFirmware/build:\nRepair completed by/date:\nTap order: sacrum, lower back, mid back, upper back, sternum\nTap schedule deviations:\nParticipant/round observations (use bout numbers):\nData-check review and decision:\n', encoding='utf-8')
    (folder / 'photos').mkdir()
    print(folder.resolve())
    return folder


def sample_info(raw):
    parts = raw.decode('utf-8', errors='replace').strip().split()
    ms = int(parts[0]) if parts and parts[0].isdigit() else None
    sensor = parts[1] if len(parts) > 1 and parts[1] in SENSORS else ''
    valid = False
    if ms is not None and sensor and len(parts) == 14:
        try:
            valid = parts[3].startswith('0x') and all(math.isfinite(float(x)) for x in parts[4:])
            int(parts[3], 16)
        except ValueError:
            valid = False
    return ms, sensor, valid


def capture_stream(ser, folder, kind, seconds):
    """Testable serial loop. Caller supplies open serial object; no reset_input_buffer."""
    folder = Path(folder)
    meta_path = folder / f'{kind}_capture.json'
    for suffix in (f'{kind}.log', f'{kind}_timing.csv', f'{kind}_capture.json'):
        if (folder / suffix).exists():
            raise ValueError(f'{suffix} already exists. Create a new session for a retry; nothing overwritten.')
    meta = {'status': 'recording', 'machine': machine(), 'started': stamp(), 'kind': kind,
            'serial_port': str(getattr(ser, 'port', 'test')), 'baud': getattr(ser, 'baudrate', None),
            'timing': 'Host receipt timestamps; MCU millis is loop timestamp, not independently verified sensor sample time.'}
    save_json(meta_path, meta, True)
    count = offset = segment = 0
    live_counts = Counter()
    next_progress = time.monotonic() + 2
    previous = None
    buffered = b''
    deadline = time.monotonic() + seconds
    try:
        with (folder / f'{kind}.log').open('xb') as rawfile, (folder / f'{kind}_timing.csv').open('x', newline='', encoding='utf-8') as timing:
            writer = csv.DictWriter(timing, fieldnames=TIMING_FIELDS); writer.writeheader(); timing.flush()
            def record(raw, ts, complete):
                nonlocal count, offset, previous, segment, next_progress
                ms, sensor, valid = sample_info(raw)
                if ms is not None:
                    if previous is not None and ms < previous:
                        segment += 1
                    previous = ms
                count += 1
                if valid and complete: live_counts[sensor] += 1
                if time.monotonic() >= next_progress:
                    print(f'LIVE lines={count} samples={dict(live_counts)} MCU_ms={ms} segment={segment}', flush=True)
                    next_progress = time.monotonic() + 2
                rawfile.write(raw); rawfile.flush()
                writer.writerow(dict(line=count, offset=offset, bytes=len(raw), complete=int(complete), **ts,
                                     segment=segment, mcu_ms='' if ms is None else ms, sensor=sensor, valid_sample=int(valid and complete)))
                timing.flush(); offset += len(raw)
            try:
                print('RECORDING. Keep this window open. Stop with Ctrl+C AFTER cues finish.', flush=True)
                while time.monotonic() < deadline:
                    chunk = ser.readline()
                    ts = stamp()
                    if not chunk:
                        continue
                    buffered += chunk
                    while b'\n' in buffered:
                        line, buffered = buffered.split(b'\n', 1)
                        record(line + b'\n', ts, True)
                meta['status'] = 'ceiling_reached'
                print('Safety time limit reached. Recording stopped; review whether the sequence completed.')
            except KeyboardInterrupt:
                meta['status'] = 'stopped_by_operator'
            except Exception as exc:
                meta['status'] = 'error'; meta['error'] = str(exc)
            finally:
                if buffered:
                    record(buffered, stamp(), False)
                os.fsync(rawfile.fileno()); os.fsync(timing.fileno())
    except Exception as exc:
        meta['status'] = 'error'; meta['error'] = str(exc)
    finally:
        meta.update(ended=stamp(), lines=count, raw_bytes=offset, timestamp_segments=segment + 1)
        save_json(meta_path, meta)
    print(f'{kind}: {count} lines saved; status={meta["status"]}.')
    return 2 if meta['status'] == 'error' else 0


def sequence(reps, seed):
    rng = random.Random(seed)
    for _ in range(10000):
        result = MOVEMENTS * reps
        rng.shuffle(result)
        if all(not (result[i] == result[i+1] == result[i+2]) for i in range(len(result)-2)):
            return result
    raise ValueError('Could not build sequence.')


def beep():
    if sys.platform == 'win32':
        try:
            import winsound
            winsound.Beep(880, 120)
        except RuntimeError:
            print('[Sound unavailable: watch the screen.]')
    else:
        print('\a', end='', flush=True)


def cues(folder, reps, seed):
    folder = Path(folder)
    recording = read_json(folder / 'imu_capture.json')
    if recording['status'] != 'recording' or recording['machine'] != machine():
        raise ValueError('Start IMU capture for this session on THIS computer first.')
    if time.time() - (folder / 'imu_timing.csv').stat().st_mtime > 10:
        raise ValueError('No recent IMU receipt file update. Check live capture before starting cues.')
    for name in ('cues.csv', 'cue_run.json'):
        if (folder / name).exists():
            raise ValueError('Cues already exist. Create a new session for a retry.')
    order = sequence(reps, seed)
    meta = {'status': 'running', 'machine': machine(), 'seed': seed, 'reps': reps, 'order': order,
            'started': stamp(), 'cadence_seconds': {'announce': 2, 'move': 5, 'neutral': 3, 'pre': 8, 'post': 8},
            'instruction': 'Operator reads displayed movement aloud. Beeps do not name the movement; cue times are prompts, not measured movement onset.'}
    save_json(folder / 'cue_run.json', meta, True)
    print('Operator: read each NEXT movement aloud immediately. Participant waits for GO. No automatic speech.')
    try:
        with (folder / 'cues.csv').open('x', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=CUE_FIELDS); w.writeheader(); f.flush()
            def event(name, bout=0, movement=''):
                w.writerow(dict(event=name, bout=bout, movement=movement, **stamp())); f.flush()
                print(f'{bout or ""} {name.upper()} {movement}', flush=True)
            event('pre_neutral'); time.sleep(8)
            for i, movement in enumerate(order, 1):
                event('next', i, movement); beep(); time.sleep(2)
                event('go', i, movement); beep(); time.sleep(5)
                event('return_neutral', i, movement); beep(); time.sleep(3)
            event('post_neutral'); time.sleep(8)
            event('complete'); os.fsync(f.fileno())
            meta['status'] = 'complete'
    except KeyboardInterrupt:
        meta['status'] = 'interrupted'
    except Exception as exc:
        meta['status'] = 'error'; meta['error'] = str(exc)
    finally:
        meta['ended'] = stamp(); save_json(folder / 'cue_run.json', meta)
    print(f'Cues {meta["status"]}. Now stop capture with Ctrl+C in the other window. Preserve all files.')
    return 0 if meta['status'] == 'complete' else 2


def read_csv(path):
    with Path(path).open(newline='', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def inspect_capture(folder, kind, errors, warnings):
    raw_path = folder / f'{kind}.log'
    rows = read_csv(folder / f'{kind}_timing.csv')
    meta = read_json(folder / f'{kind}_capture.json')
    if meta['status'] not in ('stopped_by_operator', 'ceiling_reached'):
        errors.append(f'{kind}: recording not cleanly stopped ({meta["status"]}).')
    if meta['status'] == 'ceiling_reached':
        warnings.append(f'{kind}: safety ceiling reached; check completeness.')
    stats = {'sensor_samples': Counter(), 'frames': defaultdict(set), 'max_host_gap_s': {}, 'rows': len(rows)}
    valid_rows = []
    cursor = 0
    prev_host = None
    offsets = []
    recovery = read_fail = 0
    failed_samples = Counter()
    failed_examples = {}
    expected_segment = 0
    previous_ms = None
    with raw_path.open('rb') as f:
        for row in rows:
            size = int(row['bytes'])
            if int(row['offset']) != cursor or size <= 0:
                raise ValueError(f'{kind}: broken raw/sidecar offsets')
            raw = f.read(size); cursor += size
            if len(raw) != size:
                raise ValueError(f'{kind}: raw file shorter than sidecar')
            ms, sensor, valid = sample_info(raw)
            if ms is not None:
                if previous_ms is not None and ms < previous_ms: expected_segment += 1
                previous_ms = ms
            if int(row['segment']) != expected_segment:
                raise ValueError(f'{kind}: incorrect MCU segment label')
            complete = raw.endswith(b'\n')
            if str(int(complete)) != row['complete'] or str(int(valid and complete)) != row['valid_sample'] or row['sensor'] != sensor or row['mcu_ms'] != ('' if ms is None else str(ms)):
                raise ValueError(f'{kind}: sidecar does not match raw line')
            if not complete:
                errors.append(f'{kind}: incomplete trailing serial line.')
            mono = int(row['host_monotonic_ns']); wall = int(row['host_unix_ns'])
            if prev_host is not None and mono < prev_host:
                errors.append(f'{kind}: host monotonic clock decreased.')
            prev_host = mono; offsets.append(wall - mono)
            if b'RECOVER' in raw: recovery += 1
            if b'READ_FAIL' in raw: read_fail += 1
            if sensor and not valid:
                failed_samples[sensor] += 1
                failed_examples.setdefault(sensor, row['line'])
            if valid and complete:
                stats['sensor_samples'][sensor] += 1
                stats['frames'][(int(row['segment']), ms)].add(sensor)
                valid_rows.append(row)
        if f.read(1):
            raise ValueError(f'{kind}: raw bytes have no timing sidecar entry')
    if not rows or not valid_rows:
        errors.append(f'{kind}: no valid samples.')
    missing = SENSORS - set(stats['sensor_samples'])
    if missing: errors.append(f'{kind}: missing sensors {sorted(missing)}.')
    segments = {int(r['segment']) for r in rows}
    if len(segments) > 1:
        errors.append(f'{kind}: MCU clock reset/backward jump/wrap; do not align across segments automatically.')
    # 100 ms is a computer-clock diagnostic, not a validated physiology/data-quality threshold.
    drift = (max(offsets)-min(offsets))/1e9 if offsets else 0
    if drift > .1: errors.append(f'{kind}: wall-minus-monotonic variation >100 ms; investigate clock adjustment.')
    stats['observed_receive_rate_hz'] = {}
    stats['firmware_frame_target_hz'] = 120
    for sensor, n in failed_samples.items():
        errors.append(f'{kind}: {n} invalid/failed samples for {sensor}; first line {failed_examples[sensor]}.')
    stats['invalid_samples'] = dict(failed_samples)
    for sensor in sorted(SENSORS):
        ts = [int(r['host_monotonic_ns']) for r in valid_rows if r['sensor'] == sensor]
        stats['max_host_gap_s'][sensor] = max((b-a for a,b in zip(ts,ts[1:])), default=0)/1e9
        stats['observed_receive_rate_hz'][sensor] = (len(ts)-1)*1e9/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else 0
        if stats['max_host_gap_s'][sensor] > 1:
            errors.append(f'{kind}: {sensor} host receipt gap exceeds 1 s; conservative acquisition diagnostic, not a validated scientific cutoff.')
    frames = stats.pop('frames')
    stats['incomplete_frames'] = sum(x != SENSORS for x in frames.values())
    stats['frame_count'] = len(frames)
    stats.update(recovery_lines=recovery, read_fail_lines=read_fail, wall_minus_monotonic_range_s=drift)
    if recovery or read_fail or stats['incomplete_frames']:
        warnings.append(f'{kind}: recovery/read-fail/incomplete frames require review; see counts.')
    stats['sha256_raw'] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
    return stats, valid_rows, meta


def check(folder):
    folder = Path(folder)
    if not folder.is_dir():
        print('BLOCKED: session directory does not exist.', file=sys.stderr)
        return 2
    errors, warnings = [], []
    report = {'status': 'BLOCKED', 'errors': errors, 'review_notes': warnings, 'captures': {},
              'limitations': ['Receipt times are not verified sensor sample times; USB buffering/latency remain unmeasured.',
                              'Cue timestamps label prompts, not observed movement execution.',
                              'No automatic scientific-quality PASS: inspect placement, tap mapping, photos, movements and timing.']}
    try:
        session = read_json(folder / 'session.json')
        captures = {}
        for kind in ('tap', 'imu'):
            stats, rows, meta = inspect_capture(folder, kind, errors, warnings)
            report['captures'][kind] = stats; captures[kind] = (rows, meta)
            if meta['machine'] != session['machine']: errors.append(f'{kind}: computer differs from session creator.')
        cue_meta = read_json(folder / 'cue_run.json'); events = read_csv(folder / 'cues.csv')
        imu, imu_meta = captures['imu']
        if cue_meta['status'] != 'complete': errors.append('Cue run is incomplete.')
        if cue_meta['machine'] != imu_meta['machine']: errors.append('Cue and IMU computer mismatch.')
        expected = [('pre_neutral', 0, '')]
        for i, movement in enumerate(cue_meta['order'], 1):
            expected.extend((e, i, movement) for e in ('next','go','return_neutral'))
        expected.extend([('post_neutral',0,''),('complete',0,'')])
        actual = [(r['event'],int(r['bout']),r['movement']) for r in events]
        if actual != expected: errors.append('Cue rows missing, reordered, or inconsistent with planned sequence.')
        if Counter(cue_meta['order']) != Counter({m:cue_meta['reps'] for m in MOVEMENTS}): errors.append('Cue movement counts inconsistent.')
        event_times = [int(r['host_monotonic_ns']) for r in events]
        if not event_times or any(b <= a for a,b in zip(event_times,event_times[1:])): errors.append('Cue timestamps missing/non-increasing.')
        if event_times:
            for sensor in SENSORS:
                ts = [int(r['host_monotonic_ns']) for r in imu if r['sensor'] == sensor]
                if not ts or min(ts) > min(event_times) or max(ts) < max(event_times):
                    errors.append(f'{sensor}: capture does not cover full pre-neutral through completion interval.')
            offsets = [int(r['host_unix_ns'])-int(r['host_monotonic_ns']) for r in events + imu]
            if max(offsets)-min(offsets) > 100_000_000: errors.append('Cue/IMU clock correspondence inconsistent (>100 ms); review reboot/clock changes.')
        placement = read_csv(folder / 'placement.csv')
        if Counter(r['sensor'] for r in placement) != Counter(SENSORS) or Counter(r['body_position'].strip().lower() for r in placement) != Counter(POSITIONS):
            errors.append('Fill placement.csv with each IMU0–IMU4 and each required body position exactly once.')
        photos = [p for p in (folder/'photos').iterdir() if p.is_file() and p.suffix.lower() in ('.jpg','.jpeg','.png','.heic') and p.stat().st_size]
        if not photos: errors.append('No placement photos found in photos/.')
        notes = (folder/'notes.txt').read_text(encoding='utf-8')
        if not notes.strip(): errors.append('Notes are empty.')
        warnings.append('Manually confirm operator/firmware/repair details in notes, complete placement photographs, tap order and mapping, participant movement compliance, and anomalies.')
        report['status'] = 'BLOCKED' if errors else 'REVIEW_REQUIRED'
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append(f'Missing or invalid critical file: {exc}')
    report_path = folder / f'qc_{dt.datetime.now(dt.timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:8]}.json'
    save_json(report_path, report, True)
    print(report['status'])
    for item in errors: print('BLOCK: ' + item)
    for item in warnings: print('REVIEW: ' + item)
    print(f'Report: {report_path.resolve()}')
    print('Keep and return ALL files even if blocked. Do not delete failed attempts.')
    return 2 if errors else 1


def positive(value):
    n = int(value)
    if n < 1 or n > 1000: raise argparse.ArgumentTypeError('Use an integer from 1 to 1000.')
    return n


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('ports', help='List available serial ports')
    c = sub.add_parser('create', help='Create a unique session; prints its full path')
    c.add_argument('--subject', required=True); c.add_argument('--round', type=int, required=True); c.add_argument('--root', type=Path, default=Path('data'))
    c = sub.add_parser('capture', help='Record raw bytes and host receipt timing')
    c.add_argument('--session', type=Path, required=True); c.add_argument('--serial', required=True); c.add_argument('--kind', choices=['tap','imu'], required=True)
    c.add_argument('--baud', type=int, default=921600); c.add_argument('--seconds', type=float, default=7200)
    c = sub.add_parser('cues', help='Run on same PC, after IMU recording starts')
    c.add_argument('--session', type=Path, required=True); c.add_argument('--reps', type=positive, default=10); c.add_argument('--seed', type=int, required=True)
    c = sub.add_parser('check', help='Check completed session; 1=manual review, 2=blocked, never automatic PASS')
    c.add_argument('--session', type=Path, required=True)
    a = p.parse_args(argv)
    try:
        if a.command == 'create':
            if a.round < 0: raise ValueError('Round must be 0 or greater.')
            create(a.root,a.subject,a.round); return 0
        if a.command == 'check': return check(a.session)
        if a.command == 'cues': return cues(a.session,a.reps,a.seed)
        try:
            import serial
            from serial.tools import list_ports
        except ImportError:
            raise ValueError('Install dependencies: python -m pip install -r requirements.txt')
        if a.command == 'ports':
            ports = list(list_ports.comports())
            for port in ports: print(f'{port.device}: {port.description}')
            if not ports: print('No ports found. Connect the board, check USB data cable and drivers.')
            return 0
        if not math.isfinite(a.seconds) or a.seconds <= 0: raise ValueError('--seconds must be finite and greater than zero.')
        session = read_json(a.session / 'session.json')
        if session['machine'] != machine(): raise ValueError('Create this session on the acquisition computer.')
        # Preflight before serial open: opening the port can reset some boards.
        if any((a.session/f'{a.kind}{suffix}').exists() for suffix in ('.log','_timing.csv','_capture.json')):
            raise ValueError('Recording exists. Create a new session; no files overwritten.')
        with serial.Serial(a.serial,a.baud,timeout=.2) as ser:
            return capture_stream(ser,a.session,a.kind,a.seconds)
    except (OSError, ValueError, KeyError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr); return 2

if __name__ == '__main__':
    sys.exit(main())
