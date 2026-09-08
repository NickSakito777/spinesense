#!/usr/bin/env python3
"""Offline tap trace review; manual mapping only, never automatic scientific PASS."""
from __future__ import annotations
import argparse
from collections import Counter
import datetime as dt
import html
import math
from pathlib import Path
import sys
import uuid

from session import SENSORS, sample_info


def parse_taps(path):
    traces = {sensor: [] for sensor in sorted(SENSORS)}
    invalid = Counter()
    previous_ms = None
    resets = 0
    partial = 0
    total = 0
    with Path(path).open('rb') as f:
        for raw in f:
            total += 1
            ms, sensor, valid = sample_info(raw)
            if ms is not None:
                if previous_ms is not None and ms < previous_ms:
                    resets += 1
                previous_ms = ms
            if not raw.endswith(b'\n'):
                partial += 1
                valid = False
            if sensor and not valid:
                invalid[sensor] += 1
            if valid:
                values = [float(x) for x in raw.decode('utf-8').split()[4:7]]
                magnitude = math.hypot(*values)
                if not math.isfinite(magnitude):
                    invalid[sensor] += 1
                else:
                    traces[sensor].append((ms, magnitude))
    problems = []
    if total == 0: problems.append('Empty tap log.')
    if resets: problems.append(f'{resets} backward MCU timestamp jump(s): possible reset/wrap. Traces are suppressed; do not connect separate clock segments.')
    for sensor, points in traces.items():
        if len(points) < 2: problems.append(f'{sensor}: fewer than two valid samples.')
    if invalid: problems.append(f'Invalid/nonfinite/READ_FAIL sample counts: {dict(invalid)}.')
    if partial: problems.append(f'{partial} incomplete serial line(s).')
    return traces, {'physical_lines':total,'invalid_samples':dict(invalid),'backward_jumps':resets,'partial_lines':partial,'problems':problems}


def downsample_extrema(points, max_points=2400):
    """Preserve first/last and both extrema of each contiguous index bucket."""
    if len(points) <= max_points:
        return list(points)
    buckets = max(1,(max_points-2)//2)
    interior = points[1:-1]
    width = math.ceil(len(interior)/buckets)
    result = [points[0]]
    for start in range(0,len(interior),width):
        chunk = interior[start:start+width]
        indices = sorted({min(range(len(chunk)),key=lambda i:chunk[i][1]),max(range(len(chunk)),key=lambda i:chunk[i][1])})
        result.extend(chunk[i] for i in indices)
    result.append(points[-1])
    return result


def render(traces, stats, title):
    esc = html.escape
    problems = stats['problems']
    status = 'BLOCKED — investigate recorded problems' if problems else 'MANUAL REVIEW REQUIRED'
    text = f'''<!doctype html><html lang="en"><meta charset="utf-8"><title>Tap review</title>
<style>body{{font:16px system-ui,sans-serif;max-width:1200px;margin:32px auto;padding:0 20px;color:#172334;background:#f7fafc}} svg{{width:100%;height:auto;background:white;border:1px solid #ccd5df}} h2{{margin-bottom:8px}} .notice{{padding:16px;background:#fff3cd}} li{{margin:8px 0}} table{{border-collapse:collapse}} td,th{{padding:6px 14px;border:1px solid #ccd5df}}</style>
<h1>Tap review: {esc(title)}</h1><p class="notice"><strong>{status}</strong>. This page does not determine sensor placement or certify scientific quality.</p>
<p>Recorded acceleration magnitude √(ax² + ay² + az²), in mg, from each sensor. The horizontal axis is elapsed MCU loop time from the first valid tap-log sample. Values are not gravity-subtracted. All panels use the same time axis and vertical scale. Spikes may reflect garment motion as well as direct taps.</p>
<ol><li>Locate the five groups performed in this physical order: <strong>sacrum → lower back → mid back → upper back → sternum</strong>. Each group should contain three taps. Compare timing with the field notes and actual schedule; do not infer fixed group boundaries if the operator deviated.</li><li>Compare corresponding times across all five traces. A directly tapped sensor may show a distinct impulse, but movement can propagate to neighbours. Use the full three-tap pattern, placement photographs and notes together.</li><li>Only if all five groups are unambiguous, enter the observed IMU IDs into <code>placement.csv</code>, each body position exactly once. Do not assume the firmware position strings are correct.</li><li>If groups are missing, simultaneous, clipped, ambiguous, or inconsistent with physical inspection: record the uncertainty, keep the original files, and repeat the tap check in a fresh session before formal acquisition. Do not guess a mapping.</li></ol>
<p>Display reduction preserves local minima and maxima in contiguous sample buckets, up to approximately 2,400 plotted points per sensor. Inspect raw data for precise timing. A short impulse can remain visible, but this graph does not validate its physical cause.</p>'''
    if problems: text += '<ul>'+''.join(f'<li>{esc(p)}</li>' for p in problems)+'</ul>'
    text += '<table><tr><th>Sensor</th><th>Valid samples</th><th>Magnitude range (mg)</th></tr>'
    for sensor,points in traces.items():
        extent = f'{min(y for _,y in points):.2f}–{max(y for _,y in points):.2f}' if points else 'none'
        text += f'<tr><td>{sensor}</td><td>{len(points)}</td><td>{extent}</td></tr>'
    text += '</table>'
    all_points = [point for points in traces.values() for point in points]
    if all_points and not stats['backward_jumps']:
        t0 = min(t for t,_ in all_points); t1 = max(t for t,_ in all_points)
        low = min(y for _,y in all_points); high = max(y for _,y in all_points)
        padding = max(1,(high-low)*.05); low -= padding; high += padding
        for sensor,points in traces.items():
            text += f'<h2>{sensor}</h2><svg viewBox="0 0 1100 220" role="img" aria-label="{sensor} acceleration magnitude">'
            for i in range(6):
                x=80+i*195; seconds=(t1-t0)*i/5000
                text += f'<line x1="{x}" y1="20" x2="{x}" y2="175" stroke="#e0e5eb"/><text x="{x}" y="200" text-anchor="middle" font-size="13">{seconds:.2f}s</text>'
            for i in range(4):
                y=20+i*155/3; val=high-(high-low)*i/3
                text += f'<line x1="80" y1="{y}" x2="1055" y2="{y}" stroke="#e0e5eb"/><text x="73" y="{y+4}" text-anchor="end" font-size="12">{val:.1f}</text>'
            xy=' '.join(f'{80+(t-t0)*975/max(1,t1-t0):.2f},{20+(high-y)*155/(high-low):.2f}' for t,y in downsample_extrema(points))
            text += f'<polyline points="{xy}" fill="none" stroke="#006eac" stroke-width="1.2"/></svg>'
    else:
        text += '<p><strong>No continuous traces drawn: empty data or backward MCU clock detected.</strong></p>'
    return text + f'<p>Physical serial lines: {stats["physical_lines"]}. Keep this HTML with the unchanged tap.log and tap_timing.csv. The acquisition checker must also be run; this page does not replace it.</p></html>'


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('--session',type=Path,required=True); a=p.parse_args(argv)
    try:
        folder=a.session.resolve(); traces,stats=parse_taps(folder/'tap.log')
        path=folder/f'tap_review_{dt.datetime.now(dt.timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:8]}.html'
        with path.open('x',encoding='utf-8') as f: f.write(render(traces,stats,folder.name))
        print('BLOCKED' if stats['problems'] else 'MANUAL REVIEW REQUIRED')
        for problem in stats['problems']: print(problem)
        print(f'Open this offline HTML in your browser: {path}')
        return 2 if stats['problems'] else 1
    except (OSError,ValueError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr); return 2

if __name__=='__main__': sys.exit(main())
