# Engineering review

## Original loop

```text
accelerometer + gyro
  -> complementary angle estimate
  -> error from 2 deg setpoint
  -> PID command
  -> direction pins + PWM
  -> body motion feeds back to the IMU
```

The original controller established a working feedback path, clamped the integral term and stopped motor output beyond a 15 deg error. It also had important limitations:

1. Loop timing followed sensor updates rather than an explicit control period.
2. The first time step was not initialised defensively.
3. The gyro estimate was reset to the accelerometer every second rather than continuously correcting the integrated estimate.
4. A zero command fell through the minimum-PWM branch and could become reverse PWM.
5. The derivative term was unfiltered and used error difference directly.
6. There was no explicit arm/disarm/fault state.
7. Serial output did not record all P, I and D terms for analysis.
8. Encoder feedback and repeatable disturbance tests were absent.

## v2 changes

The proposed v2 controller adds:

- A 100 Hz scheduled loop and a valid `dt` window.
- Stationary accelerometer/gyro calibration at startup.
- `angle = alpha * (angle + gyro_rate * dt) + (1-alpha) * acc_angle`.
- Derivative-on-measurement to reduce setpoint kick.
- Conditional integration when output is saturated.
- A neutral error band before minimum PWM is applied.
- Latched `FAULT` state beyond the 15 deg limit, with a deliberate reset command.
- CSV logging of raw estimates, PID terms, command and state.

## Validation boundary

Only the historical controller is associated with the video. The v2 code is compile-checked and host-tested but cannot be described as improved physical performance until it is run on the robot.

A defensible hardware test would log angle, command and wheel velocity across:

1. Multiple balance-duration trials.
2. A controlled angular release condition.
3. Repeatable impulse disturbances.
4. Encoder-based drift and velocity correction.
5. Battery-voltage and payload-position changes.
