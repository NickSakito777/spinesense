# Windows setup, wiring and firmware flashing

Complete this page once on the collection laptop. Keep the whole handoff folder
in a short local path such as `C:\SpineSense\tom`; cloud-sync software can then
copy the completed `data` folder, but collection itself should write to the
local disk.

## 1. Equipment

- repaired SpineSense garment with five STEVAL-MKI248KA sensor assemblies;
- NUCLEO-U385RG-Q and its ST-LINK USB connector;
- known-good USB **data** cable (a charge-only cable will not create a COM port);
- confirmed 2×5 garment connector pinout/repair record and matching adapter;
- Windows 10 or 11, 64 bit, with permission to install software;
- optionally, a multimeter and a second USB cable for fault isolation;
- enough free disk space for all sessions and a separate approved backup drive
  or folder.

## 2. Install Python

1. Download the 64-bit Windows installer for
   [Python 3.12.10](https://www.python.org/downloads/release/python-31210/).
   The acquisition tool requires Python 3.10 or newer; this guide fixes 3.12 so
   every operator uses the same version.
2. Run the installer. If it offers **Add python.exe to PATH**, select it.
3. Open **Start**, type `PowerShell`, and open Windows PowerShell.
4. Run:

   ```powershell
   py -3.12 --version
   ```

   A line beginning with `Python 3.12` is success. If `py` is not recognised,
   close PowerShell, reopen it, and try again. If it still fails, reinstall
   Python with the launcher/PATH option enabled. Python's official
   [Windows documentation](https://docs.python.org/3/using/windows.html)
   describes the launcher and virtual environments.

## 3. Install Arduino IDE 2 and STM32 support

1. Download and install Arduino IDE 2 from Arduino's official
   [Windows installation page](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE).
2. Download and install STM32CubeProgrammer for Windows from the official
   [STMicroelectronics page](https://www.st.com/en/development-tools/stm32cubeprog.html).
   The SWD upload method uses it. Keep the default ST-LINK driver option.
3. Open Arduino IDE. Choose **File > Preferences**.
4. Add this exact URL to **Additional boards manager URLs**:

   ```text
   https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
   ```

5. Open **Tools > Board > Boards Manager**. Search for `STM32 MCU based
   boards`, choose version **2.12.0**, and install it. The STM32 project's
   official [getting-started page](https://github.com/stm32duino/Arduino_Core_STM32/wiki/Getting-Started)
   documents the same board-manager route.

Do not substitute another STM32 core version during this handoff. Record any
compiler error before changing versions or source files.

## 4. Put the package in place and create its Python environment

Extract the handoff archive so this file is, for example,
`C:\SpineSense\tom\BUILD.md`. In PowerShell:

```powershell
Set-Location C:\SpineSense\tom
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --no-index --find-links wheelhouse -r requirements.txt
.\.venv\Scripts\python.exe tools\session.py --help
```

Always use `.\.venv\Scripts\python.exe` as shown. Activation is not required,
so every command clearly uses the same environment. The included wheelhouse
installs the pinned `pyserial==3.5` package without contacting the internet.

## 5. Verify the connector map before power

Disconnect USB and all power. Compare Southampton's confirmed repair/pinout
record with the Nucleo-side table in [README.md](README.md). Verify at least:

- 3V3 is connected only to VDD, VDDIO and CS;
- GND is connected only to GND;
- SCL goes to D15 and SDA to D14;
- TA0 lines go to D2, D4, **D6**, D8 and D9 for IMU0–IMU4;
- the adapter is fully seated and strain-relieved;
- there are no exposed conductors, obvious shorts, cracked boards or loose
  sensor modules.

If the connector map is unavailable, ambiguous, or differs from the table,
stop before power. Do not guess a 2×5 hole orientation. A person qualified to
check the repair must resolve it and record the final map.

## 6. Connect the board and identify the two USB functions

Use the Nucleo's **ST-LINK USB connector**. Windows should expose both the
programming/debug interface and a virtual COM port. Open Device Manager, expand
**Ports (COM & LPT)**, and note the new `STMicroelectronics STLink Virtual COM
Port (COMn)` entry. Unplug/replug the cable if unsure which entry is new.

You can also list candidate serial ports from PowerShell:

```powershell
.\.venv\Scripts\python.exe tools\session.py ports
```

Write the result as `COM___`. The COM number may change after using a different
USB socket. Re-run `ports` rather than assuming the old number.

## 7. Flash the connectivity sketch

1. In Arduino IDE choose **File > Open** and open
   `firmware\imu_i3c_connectivity_5\imu_i3c_connectivity_5.ino`.
2. Confirm `build_opt.h` appears in the same sketch folder. Do not move it.
3. Set the **Tools** menu exactly:

| Setting | Required value |
|---|---|
| Board | Nucleo-64 |
| Board part number | NUCLEO-U385RG-Q |
| U(S)ART support | Enabled (generic `Serial`) |
| USB support | None |
| Optimize | Smallest (`-Os` default) |
| Debug symbols and core logs | None |
| C Runtime Library | Newlib Nano (default) |
| Upload method | STM32CubeProgrammer (SWD) |
| Port | the board's COM port, if the menu requests it |

4. Click **Upload**. Wait for a successful completion message.
5. Open **Tools > Serial Monitor**, set **115200 baud**, and press the board's
   reset button once.

The banner must name TA0 as D2, D4, D6, D8 and D9. Continue with
[docs/hardware-acceptance.md](docs/hardware-acceptance.md). If the banner shows
D3/D5, you opened an obsolete sketch; stop and reopen the sketch from this
package.

## 8. Flash the streaming sketch

Do this after the unpowered inspection and connectivity sections of hardware
acceptance pass. The streaming smoke test then completes hardware acceptance:

1. **Close the Arduino Serial Monitor.** Only one program may own the COM port.
2. Open `firmware\imu_i3c_xyz_sflp\imu_i3c_xyz_sflp.ino`.
3. Keep the same board settings and click **Upload**.
4. Do not reopen Serial Monitor during Python capture. If you inspect the stream
   manually, use **921600 baud**, then close Serial Monitor before continuing.
5. Run `session.py ports` again and record the COM port.

The streaming firmware prints tab-separated data plus diagnostic lines. Each
valid sensor line contains board time, IMU identity, a firmware position label,
dynamic address, acceleration, gyroscope and quaternion fields. The firmware
position label is not proof of the garment body position; the round-specific
placement form, photos and tap capture establish that mapping.

Proceed to [docs/operator-guide.md](docs/operator-guide.md).
