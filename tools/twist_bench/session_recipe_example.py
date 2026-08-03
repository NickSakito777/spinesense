from __future__ import annotations

"""Worked example of a session recipe, run against the bench sample log.

Usage
-----
    python session_recipe_example.py

This is deliberately the *IMU-only* half of a recipe.  Block scoring against
optical motion capture needs a paired mocap file and a fitted clock, neither of
which ship with this repository -- see the README section on participant data.
What is shown here is everything up to that point, which is also everything you
can verify without a motion-capture lab:

    load  ->  reconstruct the body chain  ->  decompose twist and bend
          ->  segment movement bouts      ->  measure per-rep range of motion

The sample is a visual-guided run: hold neutral 0-8 s, move 8-28 s, return to
neutral 28-40 s.  Because the protocol is known, the segmentation result can be
checked against it -- detected bouts should land inside the move phase.  That is
the point of the example: a recipe you cannot check is a recipe you cannot trust.

To adapt this to your own recording, change LOG, LAYOUT, and the role names.
To go further and score against mocap, add a clock fit and call
``session_recipe.score_block`` -- see its docstring for what the two accuracy
families mean.
"""

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import five_imu_fusion as fiv
import session_recipe as sr

# --- what varies per session: this is the part you rewrite -------------------

LOG = HERE.parents[1] / "docs" / "sample_visual_test.log"
MARKERS = HERE.parents[1] / "docs" / "sample_visual_test_markers.json"

# Layout preset maps firmware slots to chain roles. The bench preset names its
# links bottom/low/mid/high/top; the anatomical presets use sacrum/lower/mid/
# upper/sternum. Pick the one matching how you actually mounted the sensors --
# getting this wrong produces plausible numbers for the wrong relation.
LAYOUT = "spine5-u5-top"
ROOT, TIP = "bottom", "top"
CHAIN_RELATION = "bottom_to_top"

# Identity clock: this sample has no mocap counterpart, so IMU time is its own
# reference. With a real mocap pairing these come from the sync fit.
CLOCK = {"a": 1.0, "b": 0.0}


def main() -> int:
    phases = {p["name"]: (p["start_s"], p["end_s"]) for p in json.loads(MARKERS.read_text())["phases"]}
    still_end = phases["neutral"][1]
    move_start, move_end = phases["move"]

    res = fiv.run_pipeline(LOG, fiv.make_args(layout_preset=LAYOUT, filter="vqf"))
    rate = 1.0 / float(np.median(np.diff(res.t_s)))
    print(f"loaded {LOG.name}: {len(res.t_s)} frames, {res.t_s[-1]:.1f} s, {rate:.1f} Hz measured")
    print(f"  chain relations: {sorted(res.relations)}")

    # Re-tare against the still window before the movement phase. `pre_mask`
    # takes the 1.0 s ending 0.2 s before `lo`, so pass the end of the neutral
    # hold. Everything downstream is relative to that reference pose.
    lo = still_end
    twist = sr.local_twist(res, CLOCK["a"], CLOCK["b"], lo, root=ROOT, tip=TIP)
    swing = sr.local_swing(res, CLOCK["a"], CLOCK["b"], lo, relation=CHAIN_RELATION)
    print(f"  axial twist : {twist.min():7.1f} .. {twist.max():6.1f} deg")
    print(f"  bend  swing : {swing.min():7.1f} .. {swing.max():6.1f} deg")

    # Segment bouts inside the declared move phase. Without a mocap reference
    # we segment on the IMU bend magnitude itself; with mocap you would segment
    # on the signed mocap channels via `movement_bouts` instead.
    magnitude = np.abs(swing)
    bouts = sr.peak_bouts(res.t_s, magnitude, [move_start, move_end], sign=0, min_dist_s=2.0)
    print(f"  bouts in move phase [{move_start:.0f}, {move_end:.0f}] s: {len(bouts)}, span {sr.win(bouts)}")

    # Sanity check the segmenter against the known protocol. A bout outside the
    # move phase means the detector is firing on drift or on the return motion.
    stray = [b for b in bouts if b[0] < move_start or b[1] > move_end]
    print(f"  bouts outside the move phase: {len(stray)} (expected 0)")

    troughs = sr.find_reps(res.t_s, magnitude)
    if len(troughs) > 1:
        per_rep = sr.rom(magnitude, troughs)
        print(f"  reps: {len(per_rep)}, per-rep ROM median {np.median(per_rep):.1f} deg, "
              f"spread {per_rep.min():.1f}-{per_rep.max():.1f} deg")

    print("\nNext step, with your own mocap pairing:")
    print("  1. fit the clock  t_imu = a * t_mocap + b  and check its correlation")
    print("  2. tm, sig = session_recipe.signals_for(your_mocap.csv)")
    print("  3. bouts = session_recipe.movement_bouts(tm, sig['flex'], sig['lat'], sig['axial'])")
    print("  4. session_recipe.score_block(res, clock, tm, sig['axial'], bouts[...], ...)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
