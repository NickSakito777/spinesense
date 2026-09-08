# SpineSense collection desk checklist

Print one copy per round. Full instructions remain in `protocol/session-plan.md`.

```text
Participant code: __________   Round: ____   Seed: ____   Date: __________
Operator: ___________________   COM port: ____
Session path: ____________________________________________________________

BEFORE DONNING
[ ] Hardware accepted; today's 3 stationary + 3 gentle-flex scans ALL_PASS
[ ] Streaming firmware flashed; Arduino Serial Monitor CLOSED
[ ] PC on mains; sleep disabled; disk and approved backup ready
[ ] Correct unused participant code and seed selected

THIS DONNING
[ ] Garment completely put on; cable strain-relieved
[ ] Fresh front + back + five sensor photos in this session's photos folder
[ ] placement.csv ready; mapping may remain blank until ordered taps are reviewed
[ ] notes.txt header filled; no name or identifying details

TAP CAPTURE
[ ] Tap capture says RECORDING
[ ] 3 taps each, ~2 s apart: sacrum -> lower -> mid -> upper -> sternum
[ ] Ctrl+C once; saved summary shown; deviations noted
[ ] review_taps.py run; tap_review HTML inspected; all groups unambiguous
[ ] placement.csv now maps IMU0–IMU4 to all five positions exactly once

FORMAL CAPTURE — TWO WINDOWS
[ ] Window A IMU capture says RECORDING
[ ] Live counts increase for IMU0, IMU1, IMU2, IMU3, IMU4
[ ] Ordinary comfortable-movement instruction given
[ ] Window B uses --reps 10 and correct seed
[ ] Operator reads every NEXT cue aloud; participant waits for GO
[ ] Bout deviations recorded by number
[ ] Window B prints COMPLETE; wait ~2 s; Ctrl+C Window A once

REVIEW AND BACKUP
[ ] check command run; result: BLOCKED / REVIEW_REQUIRED
[ ] Latest qc_*.json read; photos/mapping/taps/movements manually reviewed
[ ] notes decision: ACCEPT FOR RETURN / REJECT-REPEAT, initials/date
[ ] Whole directory copied to approved second location; count/size compared

REDON ROUND 2 ONLY
[ ] Round 1 finished and backed up
[ ] Garment completely removed, then put on again
[ ] New --round 2 directory, photos, placement, notes and taps
[ ] Correct round-2 seed used; independently checked and backed up
```

Stop the round for participant discomfort/pain/dizziness/unsteadiness, loss of a
sensor, USB/PC interruption or incomplete cues. Preserve all files, note the
event, fix the cause and use a new session directory for any repeat.
