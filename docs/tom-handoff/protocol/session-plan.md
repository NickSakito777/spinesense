# Rehearsal and formal session plan

Use this procedure after the repaired garment passes `hardware-acceptance.md`.
One session directory is one physical donning and one round. The target is 4–6
participants, each completing one 60-bout round, plus at least one true
remove-and-redon round.

## Movement set and ordinary instructions

The six cue names are: flexion, extension, left bend, right bend, left twist and
right twist. The operator may demonstrate the six names before capture if that
is part of the locally approved procedure, but must not coach a particular angle
or claim a clinical range.

Tell the participant before the round:

> There will be six types of trunk movement. For each bout, wait while I read
> the next movement, begin when the screen says GO, make one smooth movement
> within a range that feels comfortable and controlled, then return to your
> neutral standing position when prompted. You can make the movement smaller or
> stop at any time. Tell me immediately if you feel discomfort, pain, dizziness
> or unsteadiness.

If the participant stops or reports one of those problems, end the movement and
follow the locally approved participant procedure. Do not ask them to continue
for the sake of completing a file.

## Fixed seeds

| Participant | Round 1 | Redon round 2 |
|---|---:|---:|
| SOT01 | 101 | 102 |
| SOT02 | 201 | 202 |
| SOT03 | 301 | 302 |
| SOT04 | 401 | 402 |
| SOT05 | 501 | 502 |
| SOT06 | 601 | 602 |

The sequence is saved in `cue_run.json`. If a wrong seed is used, preserve the
session and record it; do not rename or edit the cue files. Never reuse an old
session directory.

## Mandatory rehearsal before the first participant

Use a team member or a garment stand-in. This is a complete short run, including
photos, mapping and taps. It verifies the operator, disk, serial data, receipt
timestamps, cues and QC together.

1. Flash the streaming firmware and close Arduino Serial Monitor.
2. Create the directory:

   ```powershell
   Set-Location C:\SpineSense\tom
   .\.venv\Scripts\python.exe tools\session.py create --subject CHECK --round 0 --root data
   ```

3. Copy the printed path. Fill `placement.csv` and `notes.txt`, and put at least
   front/back placement images in `photos`.
4. Run the tap calibration procedure below.
5. Open two PowerShell windows. Start `--kind imu` in Window A. After it prints
   `RECORDING`, run cues in Window B with only two repetitions:

   ```powershell
   .\.venv\Scripts\python.exe tools\session.py cues --session "PASTE_SESSION_PATH" --reps 2 --seed 100
   ```

6. Read every displayed `NEXT` cue aloud. After `COMPLETE`, wait about two
   seconds, stop capture with Ctrl+C, then run:

   ```powershell
   .\.venv\Scripts\python.exe tools\session.py check --session "PASTE_SESSION_PATH"
   ```

7. Require no `BLOCKED` result. Manually review every QC warning, photos,
   placement and taps, then record `REHEARSAL ACCEPTED`, reviewer initials and
   date in notes. If anything failed, use the troubleshooting guide and repeat
   the whole rehearsal in a newly created `CHECK` directory.

Do not start participant sessions until the rehearsal is accepted.

## Formal round: preparation

1. Confirm the day's three stationary and three gentle-flex connectivity scans
   were all `ALL_PASS`; then reflash streaming firmware and close Serial Monitor.
2. Confirm the PC is connected to mains power, automatic sleep is disabled for
   the session, local disk space is available, and an approved backup location
   is ready.
3. Confirm no names or identifying details will be entered. Allocate the next
   unused code `SOT01`–`SOT06`.
4. Ask the participant to put on the garment according to the team's established
   garment procedure. Route and strain-relieve the cable so it cannot pull the
   repaired connector during comfortable movement.
5. Create the round directory. Example for SOT01 round 1:

   ```powershell
   .\.venv\Scripts\python.exe tools\session.py create --subject SOT01 --round 1 --root data
   ```

6. Copy the printed absolute path into the paper checklist. Use only this path
   for the rest of the round.
7. Take and save fresh, face-free photos for this donning: front, back and a
   close-up of each of the five sensor positions.
8. Leave `placement.csv` blank unless the identity mapping is already supported
   by this donning's evidence. It will be completed after the ordered taps.
9. Fill the header fields in `notes.txt`.

## Tap calibration for every donning

Tap calibration is independent evidence for the mapping. It must be recorded
for every round, including redon round 2.

1. Confirm Serial Monitor is closed.
2. Start tap capture in PowerShell:

   ```powershell
   .\.venv\Scripts\python.exe tools\session.py capture --session "PASTE_SESSION_PATH" --serial COM3 --kind tap
   ```

3. Wait for `RECORDING`. Tap each physical sensor **three times**, leaving about
   two seconds between taps, in this fixed order:

   ```text
   sacrum -> lower back -> mid back -> upper back -> sternum
   ```

4. Leave about two seconds while changing to the next sensor so groups remain
   distinct. The procedure normally takes about 30–40 seconds; completeness and
   clear separation matter more than an exact total duration.
5. Press Ctrl+C once and wait for the capture summary. Record any missed,
   accidental or extra tap in `notes.txt`. If the recording is unusable, preserve
   the directory and create a new session for the whole round.
6. Generate the offline tap review:

   ```powershell
   .\.venv\Scripts\python.exe tools\review_taps.py --session "PASTE_SESSION_PATH"
   ```

7. Double-click the new `tap_review_*.html`. Compare the five plotted traces
   against the known physical order of the five tap groups. Use that evidence to
   complete IMU0–IMU4 in `placement.csv`, using each allowed body position
   exactly once. The page does not assign the map or pass it automatically. If
   the groups are not clear enough to identify every sensor, do not guess;
   preserve the session and repeat in a new directory.

## Formal 60-bout capture

1. Open two PowerShell windows in `C:\SpineSense\tom`.
2. In Window A start IMU capture, replacing the path and COM port:

   ```powershell
   .\.venv\Scripts\python.exe tools\session.py capture --session "PASTE_SESSION_PATH" --serial COM3 --kind imu
   ```

3. Wait for `RECORDING` and watch at least one live sensor-count update. All five
   sensor names must be increasing. If any is absent, stop and troubleshoot.
4. Position the participant in a safe space for the locally approved session.
   Give the ordinary instructions above.
5. In Window B run the participant's round-1 seed. Example:

   ```powershell
   .\.venv\Scripts\python.exe tools\session.py cues --session "PASTE_SESSION_PATH" --reps 10 --seed 101
   ```

6. Immediately read each displayed `NEXT` movement aloud. The participant waits
   for `GO`. The program controls the full cadence and the 8-second neutral
   periods; do not add a separate manual countdown.
7. Observe without changing the randomized order. Record any wrong, incomplete
   or interrupted bout by number in notes.
8. When Window B prints `COMPLETE`, the final 8-second neutral interval has
   finished. Wait about two seconds, then press Ctrl+C once in Window A. Wait for
   its saved summary.
9. Run QC:

   ```powershell
   .\.venv\Scripts\python.exe tools\session.py check --session "PASTE_SESSION_PATH"
   $LASTEXITCODE
   ```

10. A `BLOCKED` result or exit code 2 is not accepted. Keep the folder and decide
    whether to repeat after correcting the cause. `REVIEW_REQUIRED`/exit code 1
    means inspect every warning plus mapping, photos, tap pattern, participant
    performance and notes. Sign `ACCEPT FOR RETURN` or `REJECT/REPEAT` in notes.
11. Copy the complete session directory to the approved second location before
    starting another participant. Compare the source and copied directory sizes.

One formal round takes about 10 minutes of cue time plus setup, tap calibration,
review and backup.

## True remove-and-redon round

At least one participant completes round 2. Do it only after all available
participants have round 1, unless scheduling requires otherwise.

1. Finish, QC and back up round 1.
2. Disconnect safely and have the participant completely remove the garment.
3. Put the garment on again as a new donning. A power cycle or connector reseat
   while the garment remains on does not count.
4. Create a new session with `--round 2`. Take new photos, rebuild
   `placement.csv`, complete new notes and record a new tap calibration.
5. Run the same 60-bout procedure using that participant's round-2 seed.
6. QC and back up the new directory independently.

If time is limited, prioritize round 1 across participants while retaining at
least one real remove-and-redon round. Never shorten a formal round by changing
the six movements or ten repetitions without recording it as a protocol
deviation.
