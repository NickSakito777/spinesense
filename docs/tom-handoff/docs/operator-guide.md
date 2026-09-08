# Operator command guide

All commands below are run from `C:\SpineSense\tom` in Windows PowerShell. If
you extracted the package elsewhere, replace that path. Do not double-click the
Python file and do not activate the virtual environment.

## The six operations

```powershell
Set-Location C:\SpineSense\tom

# 1. Find serial ports
.\.venv\Scripts\python.exe tools\session.py ports

# 2. Create one new, non-overwriting directory for one donning/round
.\.venv\Scripts\python.exe tools\session.py create --subject SOT01 --round 1 --root data

# 3. Capture either the tap calibration or the formal IMU stream
.\.venv\Scripts\python.exe tools\session.py capture --session "PASTE_SESSION_PATH" --serial COM3 --kind tap
.\.venv\Scripts\python.exe tools\session.py capture --session "PASTE_SESSION_PATH" --serial COM3 --kind imu

# 4. Make the offline page for manual tap-to-IMU mapping
.\.venv\Scripts\python.exe tools\review_taps.py --session "PASTE_SESSION_PATH"

# 5. Present the timed movement cues
.\.venv\Scripts\python.exe tools\session.py cues --session "PASTE_SESSION_PATH" --reps 10 --seed 101

# 6. Create the QC report
.\.venv\Scripts\python.exe tools\session.py check --session "PASTE_SESSION_PATH"
```

Replace `COM3` with the port printed by `ports`. `create` prints an absolute,
unique folder such as:

```text
C:\SpineSense\tom\data\SOT01_r1_20260908T101530Z_a1b2c3d4
```

Copy that whole line. Paste it between quotes wherever `PASTE_SESSION_PATH`
appears. Never type a folder name from memory.

## What `create` makes

One directory represents exactly one donning and one round. It contains:

```text
session.json
placement.csv
notes.txt
photos\
```

Later commands add `tap.log`, `tap_timing.csv`, `tap_capture.json`, `imu.log`,
`imu_timing.csv`, `imu_capture.json`, `cues.csv`, `cue_run.json` and a uniquely
named `qc_*.json` report. Host receive timestamps are stored for every complete
serial line. The raw log is also preserved byte-for-byte.

The tools refuse to overwrite a capture or cue run. If a round must be repeated,
run `create` again with the same participant code and round number. The UTC time
and random suffix make the new folder unique. Preserve the failed folder and
explain the retry in both folders' notes.

## Prepare and verify the mapping

Open `placement.csv` in Notepad or Excel. Keep the header and sensor names. The
generated blank file looks like:

```csv
sensor,body_position
IMU0,
IMU1,
IMU2,
IMU3,
IMU4,
```

After recording the ordered tap calibration, run `review_taps.py` and use its
five traces to match the physical tap groups to IMU0–IMU4. Then enter each of
these labels exactly once: `sacrum`, `lower back`, `mid back`, `upper back`, and
`sternum`. Never infer the mapping from row order and never copy a previous
round's mapping without checking it. If the traces are ambiguous, stop and
repeat in a new session instead of guessing. Save as CSV, not `.xlsx`.

`review_taps.py` saves a unique offline `tap_review_*.html` file. Exit code 1
means manual review is required, which is the normal outcome. Exit code 2 means
the input is blocked by missing, empty, partial or reset data. Keep the HTML and
the original tap log/timing files in either case.

Take a front garment view, a back garment view and one clear close-up of each
sensor position. Keep faces and identifying items outside the frame. Put all
seven or more images in this session's `photos` directory with simple names such
as `front.jpg`, `back.jpg`, `sacrum.jpg` and `sternum.jpg`.

Open `notes.txt` and fill the operator, date, firmware/build and repair fields.
Use bout numbers to record any cue or movement issue; do not record names,
medical histories or other identifying information.

## Two PowerShell windows during the formal run

Open two PowerShell windows. In both:

```powershell
Set-Location C:\SpineSense\tom
```

Window A owns the COM port and writes the IMU stream. Window B presents cues.
The cue program checks that capture is live on the same computer and refuses to
start if the timing file is stale. Arduino Serial Monitor must be closed.

**Window A:**

```powershell
.\.venv\Scripts\python.exe tools\session.py capture --session "C:\SpineSense\tom\data\SOT01_r1_20260908T101530Z_a1b2c3d4" --serial COM3 --kind imu
```

Wait until it prints `RECORDING`. Keep Window A open.

**Window B:**

```powershell
.\.venv\Scripts\python.exe tools\session.py cues --session "C:\SpineSense\tom\data\SOT01_r1_20260908T101530Z_a1b2c3d4" --reps 10 --seed 101
```

The Windows beep is only an attention signal. It does not say the movement.
The operator must immediately read every displayed `NEXT` movement aloud. The
participant waits for `GO` before moving.

The cue program automatically records 8 seconds of neutral standing before the
first bout and 8 seconds after the last bout. After Window B prints `COMPLETE`,
wait about two seconds, click Window A, and press **Ctrl+C once**. Wait for the
capture summary; do not close the window while it is writing.

## What the cue times mean

Each bout is:

- `NEXT`, 2 seconds: operator reads the named movement; participant waits;
- `GO`, 5 seconds: participant performs one comfortable, controlled movement;
- `RETURN_NEUTRAL`, 3 seconds: participant returns to their neutral stance.

The timestamps record computer prompts and serial-line receipt. They are not a
measurement of the participant's exact movement onset and are not independent
sensor sample times. If the MCU millisecond counter goes backwards, the tool
starts a new timestamp segment and QC blocks the round for investigation.

## Understanding `check`

Run `check` only after both captures and the cues have finished. It always
requires manual review and therefore does not print a scientific `PASS`.

- `BLOCKED` / PowerShell exit code 2 means a critical input or coverage rule
  failed. Do not count the round as accepted; preserve it and use the decision
  tree.
- `REVIEW_REQUIRED` / exit code 1 means automated structural checks completed.
  Read every warning, inspect the photos, mapping, tap pattern and notes, then
  sign the decision in `notes.txt`.

The automated thresholds are conservative acquisition gates. They detect an
obviously incomplete capture; they do not establish scientific validity or
confirm that a participant performed a movement correctly.

To display the exit code immediately after `check`:

```powershell
$LASTEXITCODE
```
