# Two-wheel PID robot

An independent 2022–2023 Arduino balancing prototype, followed by a 2026 controller review and bench investigation of asymmetric motor response.

The feedback path combines MPU9250 X-axis tilt estimation, complementary filtering, PID control and bidirectional H-bridge motor drive. The latest bench work separates motor startup, running thresholds and speed matching before final balance tuning.

## Current status

- **Historical result:** one 29-second uncut recording shows corrective motion and periods of upright balance, with wandering and loss of stability.
- **Latest bench findings:** standalone tests confirmed both motor-driver channels and both electrical directions. Motor B still starts unreliably in one direction and the wheels respond differently during reversals.
- **Reported redevelopment:** scheduled control at approximately 100 Hz, IMU calibration, arming/fault logic, anti-windup, serial telemetry and motor/direction-specific compensation.
- **Remaining limitation:** repeatable closed-loop balancing is unresolved. Final PID tuning is paused while motor behaviour and power delivery are investigated.
- **Source availability:** the latest bench findings are documented from the supplied development summary. Its experimental firmware and raw telemetry were not supplied; the checked-in v2 remains the earlier reviewed controller.

See the [motor-characterisation notes](docs/motor-characterisation.md) for measurements, compensation decisions and the next diagnostics.

## Provenance and evidence

- **Original project:** 2022–2023, an independent learning project.
- **Public repository created:** August 2026.
- **Original evidence retained:** byte-preserved historical firmware, the 29-second uncut recording and selected build photographs.
- **Later additions:** the 2026 engineering review, v2 refactor, host-side controller model, tests and CI workflow.
- **Latest documentation update:** 9 October 2026, incorporating user-reported bench observations; the summary does not establish an exact test date.

The historical video, checked-in v2 and later bench experiments are separate records. v2 was compile-checked and host-tested; the latest summary does not establish validated balancing performance or identify an exact tested firmware revision. Encoders are present, but wheel-speed feedback remains planned.

## Hardware and historical controller

Hardware visible or recorded in the archive:

- Arduino Uno and MPU9250 IMU
- Two geared DC motors with encoders
- Unidentified dual motor driver
- Breadboard, battery pack and 3D-printed frame

| Parameter | Historical value |
|---|---:|
| Controller | PID |
| Kp / Ki / Kd | 100 / 1 / 5 |
| Target angle | 2 deg |
| Complementary weighting | 0.95 gyro / 0.05 accelerometer |
| PWM saturation | +/-255 |
| Minimum non-zero PWM | 40 |
| Integral clamp | +/-100 |
| Safety cutoff | +/-15 deg error |

These are historical values, not a final tuning recommendation.

The balancing axis uses:

```cpp
accAngle = atan2(accY, accZ) * RAD_TO_DEG;
gyroRate = mpu.getGyroX();
```

## Latest motor findings

PWM ramp tests identified strong motor/direction dependence. Motor B reverse initially needed approximately 200 PWM and sometimes failed to start. After repeated exercise and boost experiments, its approximate running/stall threshold was 100; representative minimum PWM was 115.

Three settings were separated: **startup boost** assists breakaway/reversal, **minimum PWM** sustains motion above stall, and **direction-specific scaling** adjusts steady-state speed mismatch. Equal boosts and a common minimum near 200 produced excessive or uneven response. Reversal acceleration still differs between wheels.

The [bench notes](docs/motor-characterisation.md) retain all four threshold measurements, example settings and the unresolved motor/power diagnosis.

## Repository map

- [`firmware/original/original.ino`](firmware/original/original.ino): byte-preserved historical source.
- [`firmware/v2/v2.ino`](firmware/v2/v2.ino): earlier reviewed controller with scheduled timing, calibration, logging and safety states; it does not include the latest direction-specific compensation.
- [`docs/engineering-review.md`](docs/engineering-review.md): controller audit and validation boundary.
- [`docs/motor-characterisation.md`](docs/motor-characterisation.md): condensed latest bench findings and next steps.
- [`verification/control_model.py`](verification/control_model.py): host-side model of v2 safety and PID logic.
- [`verification/test_control_model.py`](verification/test_control_model.py): zero-dependency behavioural tests.
- [`media/balance-demonstration.mp4`](media/balance-demonstration.mp4): historical 29-second demonstration.
- [Build photographs](media/assembled-front.jpg), [controller electronics](media/controller-electronics.jpg) and [side view](media/assembled-side.jpg).

Run the existing host checks with:

```bash
python -m unittest discover -s verification -v
```

Firmware CI pins the archived [`hideakitai/MPU9250` library at 0.4.8](https://github.com/hideakitai/MPU9250), which exposes the setup, update and calibration API used here. Those checks cover the checked-in v2, not the later bench experiments.

## Next steps

Measure battery and motor voltage under load, inspect or replace the suspect motor, and use encoder counts to quantify wheel-speed mismatch. Then resume staged PID tuning and log repeated balance/disturbance trials.

This project demonstrates a complete physical feedback path and a practical debugging lesson: actuator reliability and matching must be established before PID gains can be judged.
