# IMU Selection: Six Axes, Nine Axes and BNO086

> **Historical note.** This is an early sensor-selection discussion, retained as
> project history rather than current clinical or engineering guidance. Its
> statements about scoliosis, screening, therapy, and avatar behaviour describe
> the reasoning recorded at the time; they are not a medical endorsement or a
> validated clinical claim. Use [the Tom handoff](../docs/tom-handoff/README.md)
> for current acquisition instructions.

Scoliosis detection is highly sensitive to drift. In addition, connecting
multiple sensors involves real-time communication and allocation of computing
resources.

1. Sensor-related considerations

   1. Choose six-axis or nine-axis?

      **Six-axis (accelerometer + gyroscope):** can measure only tilt angles
      (roll/pitch).

      **Yaw (heading/rotation about the vertical axis) drifts over time**, so
      absolute orientation cannot be measured.

      **Nine-axis (accelerometer + gyroscope + magnetometer):** the magnetometer
      can correct drift on the yaw axis.

      Scoliosis involves not only lateral curvature but also **vertebral
      rotation**. The Schroth method places substantial emphasis on “derotation”
      breathing. If only a six-axis sensor is used, after a few minutes the
      avatar's body may inexplicably begin to “spin,” undermining the
      demonstration.

      **Without an external heading reference, a six-axis IMU can obtain yaw
      only by integrating the gyroscope, which causes unavoidable drift. A
      nine-axis IMU provides a heading reference through the magnetometer and
      can suppress drift, making it more stable for an avatar demonstration
      intended to “display trunk rotation/orientation steadily.”**

      However!

      If two adjacent IMUs experience similar yaw drift (because they are in the
      same magnetic environment), the drift may **partly cancel** when their
      relative angle is calculated. (**Estimating Relative Angles Using Two
      Inertial Measurement Units Without Magnetometers**)

      We could therefore purchase both six-axis and nine-axis sensors and
      compare their performance.

   2. Data processing

      **Hardware solution (on-chip fusion) vs software solution (raw data)**

      **Software solution (raw data → MCU):** the sensor outputs only raw
      acceleration/angular-velocity data, and the ESP32 runs a Kalman or
      complementary filter. *Risk:* there are 4–6 nodes. Can the ESP32 run six
      high-frequency filters at the same time without stalling?

      **Hardware solution (on-chip DMP/sensor hub):** the sensor contains an MCU
      and directly outputs **quaternions**.

      - *Advantage:* the ESP32 only moves data, CPU usage is extremely low, and
        an algorithm tuned by the manufacturer is generally more stable than
        one written in-house.

      One device could satisfy both the six-axis and nine-axis tests:

      BNO086, which can also perform the solution in hardware.

      The LSM6HG256X provided by Tom
