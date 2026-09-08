# Fillable text templates

`session.py create` generates the live forms. Use these copies only to restore a
file that was accidentally left blank; never replace raw capture or timing files.

## placement.csv

Enter the true mapping for this donning. Use all five allowed labels exactly
once: `sacrum`, `lower back`, `mid back`, `upper back`, `sternum`.

```csv
sensor,body_position
IMU0,
IMU1,
IMU2,
IMU3,
IMU4,
```

## notes.txt

```text
Operator:
Date:
Firmware/build:
Repair completed by/date:
Tap order: sacrum, lower back, mid back, upper back, sternum
Tap schedule deviations:
Participant/round observations (use bout numbers):
READ_FAIL/RECOVER/USB/PC events:
Cue reading or movement deviations:
Data-check review and decision:
Reviewer initials/date:
```

## hardware acceptance

```text
Date/time:
Operator:
Person who completed/verified the repair:
Confirmed garment connector-map document/photo:
Nucleo board ID or label:
USB cable ID/description:
Visible repair points:
Connectivity: three stationary ALL_PASS scans: YES / NO
Connectivity: three gentle-flex ALL_PASS scans: YES / NO
Streaming smoke-test session path:
All IMU0–IMU4 repeatedly present: YES / NO
READ_FAIL count observed:
RECOVER lines observed:
Decision: ACCEPT / REJECT
Reason and follow-up:
```
