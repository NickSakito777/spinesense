# Check, back up and return the data

Return every attempt, including rehearsal, blocked and repeated sessions. UCL
needs the evidence of what happened and the tools never overwrite an earlier
attempt.

## End-of-round review

For each session directory, confirm all of these exist and are non-empty:

- `session.json`, `placement.csv`, `notes.txt` and `photos\`;
- `tap.log`, `tap_timing.csv`, `tap_capture.json`;
- at least one `tap_review_*.html` used for the manual mapping check;
- `imu.log`, `imu_timing.csv`, `imu_capture.json`;
- `cues.csv`, `cue_run.json`;
- at least one `qc_*.json` report.

Open the latest QC report in Notepad and compare it with the console result.
`BLOCKED` is retained and reported as blocked. `REVIEW_REQUIRED` still needs a
human decision recorded in notes; it does not mean the data are scientifically
approved.

Check the human evidence:

- `placement.csv` contains IMU0–IMU4 and all five body positions exactly once;
- photos belong to this donning and show front, back and each sensor position
  without a face;
- the tap groups follow sacrum, lower back, mid back, upper back, sternum;
- notes identify any recovery, read failure, interruption, wrong bout, repeat or
  deviation using the participant code and bout number;
- the cue run is complete and contains six movements × ten repetitions for a
  formal round;
- all five sensors cover the complete cue interval according to QC.

Do not edit generated CSV/JSON/log files to make a check pass. If a form needs a
correction, preserve the mistaken value in notes and make the factual correction
in the form before running a new QC report.

## Immediate backup

After every round, copy the whole unique session directory to the approved
second drive or institutional storage folder. Do this before the next
participant. Do not rely on a sync icon alone: open the destination and compare
the source and destination directory item counts and sizes.

Keep collection on the local PC while recording. A network or cloud folder may
pause, lock or delay rapidly updated timing files.

## Privacy check

Before packaging:

- search filenames, notes and forms for participant names or other identifying
  details and remove them from the return copy in accordance with the approved
  project procedure;
- confirm participant codes are `SOT01`–`SOT06`;
- confirm faces and identifying backgrounds are excluded from photos;
- do not put participant data in this source repository;
- use only the Southampton/UCL approved storage and transfer route.

Do not upload through a public file-sharing link. This guide does not send any
files automatically.

## Expected return tree

Exact session directory timestamps and suffixes will differ:

```text
data\
  hardware_acceptance_2026-09-08.txt
  CHECK_r0_20260908T090000Z_ab12cd34\
    session.json
    placement.csv
    notes.txt
    photos\...
    tap.log
    tap_timing.csv
    tap_capture.json
    tap_review_20260908T090500Z_1234abcd.html
    imu.log
    imu_timing.csv
    imu_capture.json
    cues.csv
    cue_run.json
    qc_20260908T091000Z_1234abcd.json
  SOT01_r1_20260908T100000Z_...
  SOT01_r2_20260908T150000Z_...
  SOT02_r1_...
```

Include any extra failed-attempt directories and explain their relationship in
their notes. Include the hardware acceptance record and the final confirmed
garment connector-map record/photo if it is approved for return.

## Create and verify the ZIP offline

Close every capture/Arduino window first. In File Explorer, right-click the
`data` folder and choose **Compress to ZIP file** (or **Send to > Compressed
(zipped) folder**, depending on Windows version). Name it with the collection
date, for example `Southampton_SpineSense_data_2026-09-08.zip`.

Open the ZIP in File Explorer without extracting it. Browse into at least the
rehearsal, one formal round and the redon round; confirm that photos, raw logs,
timing CSVs, cues and QC reports are visible. Compare the ZIP size with the data
folder and retain both the original and the separate backup until UCL confirms
receipt and readability through the approved project channel.

Alongside the ZIP, provide this plain-text manifest:

```text
Collection dates:
Operator(s):
Hardware acceptance filename and result:
Rehearsal session path and review decision:
Formal round directories:
Blocked/failed/repeated directories and reasons:
Participant codes with round 1:
Participant code(s) with true remove-and-redon round 2:
Known hardware or timing issues:
ZIP filename:
ZIP size:
Backup location checked by/date:
```
