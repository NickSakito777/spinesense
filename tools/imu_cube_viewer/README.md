# SpineSense IMU Cube Viewer

Local browser viewer for the five-IMU I3C smoke test.

## Run

1. Upload `SpineSense FYP/firmware/imu_i3c_xyz/imu_i3c_xyz.ino`.
2. Close Arduino Serial Monitor so `COM3` is free.
3. Run:

```powershell
& ".\tools\imu_cube_viewer\run_cube_viewer.cmd"
```

4. Open:

```text
http://127.0.0.1:8765
```

## Demo mode

```powershell
& python ".\tools\imu_cube_viewer\serial_bridge.py" --demo --port 8765
```

## Twist bench live view

Upload `SpineSense FYP/firmware/imu_i3c_xyz/imu_i3c_xyz.ino`, close Arduino Serial Monitor, then run:

```powershell
& ".\tools\imu_cube_viewer\run_twist_viewer.cmd"
```

Open:

```text
http://127.0.0.1:8765/twist.html
```

Default mapping:

```text
IMU1 = parent / bottom fixed block
IMU2 = child / top moving block
```

Hold the blocks still, click `Bias 3s`, then click `Tare`. After that, rotate the top block around the pillar and watch the upper cube and twist gauge.

Modes:

```text
Full 3D = integrate the three-axis relative gyro vector and drive the top cube with a quaternion.
Twist = old single-axis twist gauge for +/-30 degree checks.
```

`Full 3D` is gyro-only for now, so it is best for short live checks after `Bias 3s` and `Tare`.
