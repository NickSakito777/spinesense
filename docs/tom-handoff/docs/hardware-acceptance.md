# Repaired-garment hardware acceptance

Complete this before the rehearsal and repeat the quick connection test at the
start of every collection day. Do not put the garment on a participant until
all stop conditions below are resolved.

## Record

```text
Date/time:
Operator:
Person who completed/verified the repair:
Confirmed garment connector-map document/photo:
Nucleo board ID or label:
USB cable ID/description:
Visible repair points:
Connectivity result:
Gentle-flex result:
Streaming smoke-test session path:
Hardware smoke-test review (formal QC is N/A until the cue rehearsal):
Decision: ACCEPT / REJECT
Reason and follow-up:
```

Save this as `data\hardware_acceptance_YYYY-MM-DD.txt`. Attach the confirmed
connector-map image or record where it is stored. The received-garment photos
from 11 August 2026 show the pre-repair garment only; they do not prove the final
connector pinout or repaired state.

## A. Unpowered inspection

Disconnect USB. Check the garment, adapter and five sensor modules:

- five modules are present and held securely;
- the repaired points and connector have strain relief;
- no bare conductor, scorched part, liquid or visibly cracked board is present;
- the confirmed 2×5 connector map is available and its physical orientation is
  unambiguous;
- the adapter routes signals to D15, D14, 3V3, GND and TA0 D2/D4/D6/D8/D9 as
  listed in `README.md`;
- no cable will pull across or press into the participant during movement.

**Reject and stop before power** if any item fails, if the connector map is only
a guess, or if 3V3/GND identity is uncertain. Hardware continuity, resistance
or short-circuit measurements should be performed only by someone competent to
make them and interpreted against the repair record.

## B. Five-sensor connectivity test

Flash the connectivity sketch as described in `BUILD.md`. Open Serial Monitor at
115200 baud and reset the board. A scan repeats approximately every three
seconds.

1. Confirm the banner lists D2, D4, D6, D8 and D9.
2. Watch at least three consecutive scans without touching the garment.
3. Every scan must identify all five IMUs and end `RESULT: ALL_PASS`.
4. While watching at least three more scans, gently flex the cable bundle and
   repaired connector through the small amount of movement it will encounter in
   normal wear. Do not pull a sensor or sharply fold the conductive textile.
5. All flex-test scans must still end `RESULT: ALL_PASS`.

Any missing IMU, `WHO_AM_I` mismatch, initialization failure or intermittent
result is a rejection. Power down, inspect/reseat only the accessible connector,
then repeat the entire test. If failure follows garment movement or returns
after reseating, the repair requires investigation; do not collect around it.

## C. Streaming smoke test

Flash the streaming sketch, close Serial Monitor, and create a `CHECK` session:

```powershell
Set-Location C:\SpineSense\tom
.\.venv\Scripts\python.exe tools\session.py create --subject CHECK --round 0 --root data
```

Paste the printed directory into these commands. First record taps as specified
in the session plan and build its offline review page, then record a 60-second
stream while a wearer or garment stand remains still and the operator gently
moves each sensor one at a time. Stop each capture with Ctrl+C.

```powershell
.\.venv\Scripts\python.exe tools\session.py capture --session "PASTE_SESSION_PATH" --serial COM3 --kind tap
.\.venv\Scripts\python.exe tools\review_taps.py --session "PASTE_SESSION_PATH"
.\.venv\Scripts\python.exe tools\session.py capture --session "PASTE_SESSION_PATH" --serial COM3 --kind imu --seconds 60
```

For this hardware smoke test, cue files are not expected, so inspect the capture
console, logs and generated `tap_review_*.html` manually. Confirm all five
sensor counts rise, each physical tap group appears clearly in one sensor trace,
and no sensor disappears for an interval. Record every `READ_FAIL` and `RECOVER`
line in the acceptance record. The tap review is a visualization for a human;
it does not assign positions or issue a pass automatically.

Accept the hardware only after sections A–C are satisfactory. The formal
rehearsal in the session plan is still required because it verifies the complete
capture-and-cue chain.
