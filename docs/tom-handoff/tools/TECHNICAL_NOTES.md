# Acquisition tool interface and validation limits

Use `session.py` for this handoff. The other two Python files are preserved historical copies and do not implement the new timestamp/anti-overwrite contract. Follow the main handoff tutorial for field operations.

## Commands

Run from the `tom` directory after installing `requirements.txt` with Python 3.10 or later:

```
python tools/session.py ports
python tools/session.py create --subject SOT01 --round 1 --root data
python tools/session.py capture --session "FULL_PATH_PRINTED_BY_CREATE" --serial COM3 --kind tap
python tools/session.py capture --session "FULL_PATH_PRINTED_BY_CREATE" --serial COM3 --kind imu
python tools/session.py cues --session "FULL_PATH_PRINTED_BY_CREATE" --reps 10 --seed 101
python tools/session.py check --session "FULL_PATH_PRINTED_BY_CREATE"
```

Run tap separately and stop it before starting IMU capture. Run IMU capture and cues simultaneously in separate terminals on the same computer. Capture defaults to 921600 baud and a 7200-second safety ceiling; `--seconds` changes the ceiling. Stop with Ctrl+C. Cues automatically include 8 seconds of neutral standing before and after the sequence. After `COMPLETE`, wait about 2 seconds for final samples before stopping capture. The operator reads each NEXT movement aloud. Windows emits a beep, not speech. No MoCap is used by these commands.

A fresh session is mandatory for retrying a capture kind or cues. Existing files are never reused. `create` includes timestamp and random suffix in the folder name. No command automatically reconnects after serial failure.

## Outputs

- `session.json`: pseudonymous participant ID, don/doff round, creation clocks and computer identity.
- `placement.csv`: editable sensor/body-position mapping; `notes.txt`: editable operator, firmware, repair and observation fields; `photos/`: per-round placement images.
- `tap.log`, `imu.log`: unmodified serial bytes, including diagnostic messages and partial trailing bytes.
- `tap_timing.csv`, `imu_timing.csv`: one row per physical line, including an incomplete final line if present. Columns: `line,offset,bytes,complete,host_unix_ns,host_monotonic_ns,segment,mcu_ms,sensor,valid_sample`.
- `tap_capture.json`, `imu_capture.json`: port, baud, start/end clocks, counts and terminal status. Status can be `recording`, `stopped_by_operator`, `ceiling_reached`, or `error`. A killed process can leave `recording`; QC blocks it.
- `cues.csv`: each pre-neutral, NEXT, GO, RETURN_NEUTRAL, post-neutral and completion event is immediately written with Unix and monotonic nanoseconds. Times denote software prompt emission, not actual movement onset or completion.
- `cue_run.json`: seed, planned order, cadence, machine and final status (`running`, `complete`, `interrupted`, `error`).
- `qc_<UTC>_<suffix>.json`: unique QC report, counts, observed receipt rates, maximum gaps, raw SHA-256 hashes, blocking findings and manual review notes. Keep all attempts and all reports.

## Timing interpretation

The MCU `millis()` field is the firmware loop timestamp. Host times are read when `readline()` returns. They are receipt times, not measurements of sensor acquisition time. Serial transmission, USB/OS buffering and scheduling can delay them. Multiple complete lines arriving in one read share the receipt timestamp. A partial line assembled across reads receives the time of the read that completed it. Raw bytes remain unchanged.

A decreasing MCU timestamp starts a new segment (possible reset, wrap or other backward jump). The checker recomputes segment boundaries from raw text, checks sidecar integrity and blocks a multi-segment recording from automatic alignment. It does not fit or assert a scientifically valid clock transform. Wall-minus-monotonic variation greater than 100 ms is blocked as a computer-clock diagnostic. This is not an empirically validated motion-analysis threshold. Running the tools on the same computer is necessary, but does not prove sufficient alignment accuracy.

QC reports the firmware's approximate 120 Hz frame target alongside observed per-sensor receipt rates. Any per-sensor host receipt gap over 1 second is blocked as a conservative operational diagnostic. Shorter gaps are **not** thereby scientifically acceptable. All incomplete-frame and recovery counts, overall timing, tap mapping and movement execution require review. Raw diagnostics such as READ_FAIL or malformed/nonfinite sensor records block the round. A regular sensor stream does not establish sensor accuracy.

## Exit codes and limits

Ordinary commands: `0` means command completed; `2` means input/recording error or interrupted cue run. A safety-ceiling capture can return `0` but is marked `ceiling_reached`; inspect QC and sequence coverage.

`check`: `2 = BLOCKED`; `1 = REVIEW_REQUIRED`. It never returns an automatic scientific-quality PASS. REVIEW_REQUIRED means automatic checks found no blocking issue; placement photos, completed notes, tap interpretation, actual movement compliance, firmware/hardware identity and timing suitability still require human inspection. Photos are checked only for file presence, not image content. Missing tap/cue/timing files, unmapped placement, missing photos or incomplete sensor coverage block the combined round.

The synthetic tests do not validate Windows, physical serial throughput, repaired hardware, optical timing, sensor placement or research outcomes. Run a real Windows trial before participant acquisition. The main field guide defines the local operational review; research acceptance remains a separate decision.

## Software tests

```
python -m unittest discover -s tests -v
```

Tests use fake serial input and synthetic records. They exercise byte preservation, fragmented input, invalid UTF-8, incomplete trailing data, output collision protection, serial disconnection, MCU reset segmentation, missing critical data/sensors, sidecar tampering, cue completion/interruption, clock jumps, long receipt gaps, computer mismatch and seeded movement balance.

## Offline tap visualization

After stopping tap capture, run:

```
python tools/review_taps.py --session "FULL_PATH_PRINTED_BY_CREATE"
```

Double-click the printed `tap_review_<UTC>_<suffix>.html` to open it offline in a browser. It shows five common-scale SVG traces of acceleration magnitude (mg) against elapsed MCU loop time. Min/max-preserving reduction keeps local impulses visible without embedding every sample. No external assets, libraries or network are required. Identify the five ordered three-tap groups using the field notes, photographs and physical inspection; the tool does not assign body positions. Ambiguous coupled motion must not be treated as a confirmed mapping.

Exit `1` means manual review required; `2` means blocked/missing/invalid input. Invalid sample counts and partial lines are explicit. A backward MCU timestamp suppresses the graphs to prevent joining distinct clock segments. This view does not replace `session.py check` or validate receipt-time alignment. Keep the HTML with all original data. Synthetic tests cover five-sensor parsing, empty/invalid/partial input, reset suppression and preservation of positive/negative impulses during downsampling.
