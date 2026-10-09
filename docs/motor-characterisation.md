# Motor characterisation and PID debugging

Documentation updated: 9 October 2026. Condensed from the latest user-supplied development summary; exact test dates, raw logs and the experimental firmware revision were not supplied.

## Scope

The MPU9250 balancing axis is X: accelerometer tilt is `atan2(accY, accZ)` in degrees and angular rate is `getGyroX()`. Complementary filtering feeds a PID command and bidirectional H-bridge drive.

The checked-in [v2](../firmware/v2/v2.ino) contains the earlier approximately 100 Hz timing, calibration, filtering, arming/fault, anti-windup and telemetry work. It still sends equal PWM to both motors with a shared minimum of 40. The direction-specific compensation below describes later bench experiments; their source is not present.

## Direction debugging

The initial symptom was that opposite tilts appeared to drive different wheels. A standalone test confirmed both H-bridge channels and both electrical motor directions. Software direction mapping was then adjusted for the mounting and wiring.

The reported experimental configuration was:

```cpp
constexpr float MOTOR_COMMAND_SIGN = -1.0f;
constexpr bool MOTOR_A_REVERSED = true;
constexpr bool MOTOR_B_REVERSED = true;
```

These settings apply to this build's wiring and mounting. The standalone test supports basic channel operation, but does not rule out an intermittent driver, connection or loaded power-delivery fault.

## Approximate PWM observations

| Motor / direction | Initial breakaway threshold | Later running/stall threshold | Representative minimum |
|---|---:|---:|---:|
| A forward | 110 | 80 | 95 |
| A reverse | 95 | 70 | 85 |
| B forward | 115 | 70 | 85 |
| B reverse | ~200; sometimes no start | 100 | 115 |

Values use the 0–255 PWM command scale. Starting and running thresholds are distinct measurements. The later observations followed repeated direction changes and startup assistance; test conditions, sample counts and repeatability were not recorded in the supplied summary.

Motor B reverse sometimes required touching or tapping the motor/encoder assembly to start. Although behaviour improved after exercising the motors, intermittent starting remained unresolved.

## Separate startup, running and speed compensation

| Parameter | What it adjusts | Tuning decision |
|---|---|---|
| Startup boost PWM and duration | Breakaway and reversal response | Use a brief, non-blocking pulse, independently set per motor and direction |
| Minimum PWM | Sustained motion above stall | Add a margin above each measured running threshold |
| PWM scale | Steady-state wheel-speed mismatch | Tune forward and reverse independently |

The summary gives PWM 255 for 50–100 ms as an example of assistance for the weak Motor B reverse direction. It is an experimental example, not a final calibrated setting. Applying identical boosts to every direction worsened mismatch because healthy directions accelerated sooner.

A shared minimum near 200 was rejected: even small PID corrections then demanded high effort. Raising a minimum also cannot change a command that already exceeds it; a command of 160 stays at 160 when the floor changes from 95 to 110.

Scaling changes the requested effort above the floor. An illustrative A-forward scale of 0.95 relative to B-forward at 1.00 reduces A's PWM by 5%; it does not prove a 5% speed reduction. Scaling is open-loop compensation and still needs measured wheel speeds.

Even when eventual speeds looked similar, unequal acceleration caused rotation during PID reversals. Separate boost magnitudes and durations were therefore explored for transient matching.

## Unresolved hardware diagnosis

Motor B's direction dependence, intermittent no-start behaviour, response to tapping and improvement after exercise suggest an electrical or mechanical problem. Possible causes include brush/commutator contact, gearbox friction, terminal solder joints or connections near the encoder assembly. None is confirmed.

Power delivery is also unresolved. PWM 255 commands full duty cycle; actual motor voltage depends on the battery, driver and wiring.

Next measurements:

1. Battery voltage unloaded.
2. Battery voltage with the commanded motor stalled.
3. Voltage directly across Motor B under the same condition.

Adequate voltage with persistent stalling would implicate the motor or mechanical load; substantial voltage collapse would direct the investigation toward battery, driver or wiring. Software compensation has not established reliable bidirectional operation.

## Encoder feedback and PID status

Encoder counting and wheel-speed comparison are planned, not implemented in the supplied record. Measured left/right velocity would support a small wheel-speed correction loop alongside the balance controller, replacing reliance on visual matching and fixed scales.

Final PID tuning is paused because asymmetric actuation obscures the balance response. The intended sequence is to begin with Ki = Kd = 0, tune Kp, add Kd damping, refine Kp/Kd and the balance setpoint, then add a small Ki only if needed.

No new balance-duration distribution, disturbance-recovery result or repeatability claim is supported by this summary. The historical video remains the available recorded balancing observation.
