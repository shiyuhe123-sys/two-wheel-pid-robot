# Two-wheel PID robot

An independent 2022/23 learning prototype that closed the loop between an MPU9250 attitude estimate, a PID controller and two geared DC motors on an Arduino Uno.

## Provenance

- **Originally completed:** 2022–2023 as an independent learning project.
- **Public repository created:** August 2026.
- **Original evidence retained:** byte-preserved historical firmware, the 29-second uncut recording and selected build photographs.
- **Later portfolio additions:** the 2026 engineering review, v2 refactor, host-side controller model, tests and CI workflow.
- **Validation rerun status:** the original hardware result has not been rerun; v2 is compile-checked and host-tested but remains unvalidated on the robot.

This repository is a curated publication of the original project files plus clearly separated later engineering work. It does not imply historical commit activity or improved physical performance.

## Evidence

- One **29-second uncut recorded observation** shows active posture correction and periods of upright balance.
- The prototype also wanders and loses stability; it was not tested across repeated controlled trials.
- The motors included encoders, but the historical controller did not use them.

This repository preserves the historical source and adds a separate 2026 engineering review. The v2 firmware is a proposed refactor and is **not hardware validated**.

## Historical controller

| Parameter | Value |
|---|---:|
| Controller | PID |
| Kp / Ki / Kd | 100 / 1 / 5 |
| Target angle | 2 deg |
| Complementary weighting | 0.95 gyro / 0.05 accelerometer |
| PWM saturation | +/-255 |
| Minimum non-zero PWM | 40 |
| Integral clamp | +/-100 |
| Safety cutoff | +/-15 deg error |

Hardware visible or recorded in the project archive:

- Arduino Uno
- MPU9250 IMU
- Two geared DC motors with unused encoders
- Unidentified dual motor driver
- Breadboard, battery pack and 3D-printed frame

## Repository map

- [`firmware/original/original.ino`](firmware/original/original.ino): byte-preserved historical source.
- [`firmware/v2/v2.ino`](firmware/v2/v2.ino): reviewed controller with deterministic timing, calibration, logging and explicit safety states.
- [`docs/engineering-review.md`](docs/engineering-review.md): original-versus-v2 analysis and validation boundary.
- [`verification/control_model.py`](verification/control_model.py): host-side model of the v2 safety and PID logic.
- [`verification/test_control_model.py`](verification/test_control_model.py): zero-dependency behavioural tests.
- [`media/balance-demonstration.mp4`](media/balance-demonstration.mp4): original 29-second recorded demonstration.
- [`media/assembled-front.jpg`](media/assembled-front.jpg), [`media/controller-electronics.jpg`](media/controller-electronics.jpg) and [`media/assembled-side.jpg`](media/assembled-side.jpg): selected build photographs.

Run the host checks with:

```bash
python -m unittest discover -s verification -v
```

The firmware CI pins the archived [`hideakitai/MPU9250` library at 0.4.8](https://github.com/hideakitai/MPU9250) because that version exposes the `setup`, `update` and calibration API used here.

## What this project demonstrates

The valuable evidence is not perfect balance. It is the complete physical feedback path—sense, estimate, control and actuate—plus an explicit understanding of what must be measured in a rigorous next iteration.
