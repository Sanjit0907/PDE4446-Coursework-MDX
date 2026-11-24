# Testing and results

Every entry below is a test that was actually run. Rows marked *not yet run* are planned and have
no invented data. Raw serial captures are kept locally; the figures quoted here are copied from
them.

Hardware under test: TI-RSLK MAX chassis, Arduino Uno, 6 x AA pack.
Firmware: the stage sketches in `src/`. Tests T0 to T2 were run on stages 0 to 2; later tests use
stage 3C or stage 4 as noted.
Serial: 115200 baud.

## Summary

| # | Test | Setup | Result | Verdict |
|---|---|---|---|---|
| T0 | Motor direction and encoder counting | Wheels lifted, `teleop_test` | Both motors run forward on command. Left encoder counted backwards, corrected with `LEFT_ENC_INVERTED = true` | Pass |
| T1 | PI speed tracking, no load | Wheels lifted, target 120 mm/s | Both wheels held 118-122 mm/s. PWM settled at 28-30. Tick counts within 4 of each other | Pass |
| T2 | PI speed tracking, on the floor | Floor, target 120 mm/s, USB attached | Start transient peaked at 195 mm/s, settled to 119 mm/s within about 8 telemetry samples. Tick counts within 1.4 % | Pass at steady state |
| T3 | Straight-line travel, cable attached | Floor, 3 runs, USB cable routed to the right | Drifted right on all three runs | **Fail** |
| T4 | Straight-line travel, cable removed | Floor, approximately 1 m run | Travelled smoothly and straight. Lateral deviation not measured | Pass, qualitative only |
| T5 | Three-stop sequence timing | Not yet run | - | - |
| T6 | Ultrasonic distance accuracy | Not yet run | - | - |
| T7 | Detection and speed reduction | Not yet run | - | - |
| T8 | Avoidance manoeuvre, no obstacle | Not yet run | - | - |
| T9 | Avoidance manoeuvre with the cone | Not yet run | - | - |
| T10 | Full route, repeatability | Not yet run | - | - |

## T1 - PI speed tracking with the wheels lifted

Target 120 mm/s. Representative telemetry:

```
TARGET,120.00,VL,122.17,VR,122.17,PWM_L,29,PWM_R,28,TICKS_L,6126,TICKS_R,6129
TARGET,120.00,VL,119.12,VR,119.12,PWM_L,29,PWM_R,29,TICKS_L,6204,TICKS_R,6208
TARGET,120.00,VL,117.94,VR,117.94,PWM_L,30,PWM_R,29,TICKS_L,6282,TICKS_R,6285
TARGET,120.00,VL,122.17,VR,119.12,PWM_L,29,PWM_R,29,TICKS_L,6361,TICKS_R,6364
```

**Observations**

- Both wheel speeds stayed within 2 % of the 120 mm/s target.
- The steady-state PWM of 28-30 is a useful measurement in its own right: it is the output needed
  to hold 120 mm/s, and it shows the fixed `BASE_PWM = 65` in the controller is more than twice
  what the motors actually require at this speed.
- Over the four samples the left wheel accumulated 235 counts and the right 235 counts.

**Limitation** - an unloaded wheel does not reproduce the friction and load of driving on a floor.

## T2 - PI speed tracking on the floor

Target 120 mm/s, USB cable attached. Representative telemetry, from the start of the run:

```
TARGET,120.00,VL,128.28,VR,109.96,PWM_L,28,PWM_R,38,TICKS_L,8608,TICKS_R,8476
TARGET,120.00,VL,183.26,VR,183.26,PWM_L,55,PWM_R,55,TICKS_L,8744,TICKS_R,8604
TARGET,120.00,VL,195.48,VR,192.42,PWM_L,42,PWM_R,43,TICKS_L,8887,TICKS_R,8744
TARGET,120.00,VL,177.15,VR,167.99,PWM_L,40,PWM_R,43,TICKS_L,9002,TICKS_R,8853
TARGET,120.00,VL,155.77,VR,149.66,PWM_L,36,PWM_R,40,TICKS_L,9217,TICKS_R,9059
TARGET,120.00,VL,137.44,VR,137.44,PWM_L,34,PWM_R,38,TICKS_L,9408,TICKS_R,9246
TARGET,120.00,VL,119.12,VR,119.12,PWM_L,31,PWM_R,32,TICKS_L,10241,TICKS_R,10086
TARGET,120.00,VL,119.12,VR,119.12,PWM_L,31,PWM_R,32,TICKS_L,11184,TICKS_R,11033
```

**Observations**

- The first samples are a start-up transient, not steady-state performance. Peak measured speed was
  195 mm/s against a 120 mm/s target, an overshoot of **63 %**.
- Cause: the controller applies a fixed feedforward of `BASE_PWM = 65` while T1 showed the motors
  need approximately 29 to hold the target. Adding `Kp * error` on top at start-up commands roughly
  three times the necessary output.
- After settling, both wheels held 119 mm/s with PWM 31 and 32.
- Between the last two samples the left wheel accumulated 943 counts and the right 947, a
  difference of 0.4 %.

**Conclusion** - the telemetry does **not** show a persistent wheel-speed imbalance large enough to
explain a consistent rightward drift. The physical cause was therefore investigated separately
rather than by changing the PI gains.

## T3 and T4 - straight-line travel, and the cable-drag finding

| Run | Cable | Observation |
|---|---|---|
| 1 | Attached, routed right | Drifted right |
| 2 | Attached, routed right | Drifted right |
| 3 | Attached, routed right | Drifted right |
| 4 | Removed from the robot | Travelled smoothly and straight for approximately 1 m |

**Conclusion** - the USB cable pulling sideways on the chassis was the cause of the drift, not a
control error. This is consistent with T2, where the measured wheel speeds were balanced. No change
was made to the PI gains or the motor direction flags.

**Action taken** - physical tests are now run with the cable detached from the robot. If serial
output is needed during a run, the cable must be supported so it exerts no force on the chassis.

**Still outstanding** - T4 is qualitative. A measured deviation over the full 4.4 m route,
repeated at least three times, is needed for the report. See T10.

## Calibration still required

`COUNTS_PER_REV = 720` and the 70 mm wheel diameter are derived values. Until they are measured,
every figure quoted in millimetres above is unverified in absolute terms, although the
left-versus-right comparisons that the tests rely on remain valid.

### C1 - distance scale

1. Mark 1000 mm on the floor with tape.
2. Send `r` to zero the encoder counts.
3. Drive the robot from the start mark to the end mark and send `x`.
4. Record `TICKS_L` and `TICKS_R`.

```
MM_PER_TICK    = 1000 / mean(TICKS_L, TICKS_R)
COUNTS_PER_REV = pi * 70 / MM_PER_TICK
```

Repeat three times and record the spread. Data goes in `tests/encoder_calibration.csv`.

### C2 - turn scale

1. Place the robot against a straight edge and mark its heading.
2. Command one 90 degree pivot.
3. Measure the angle actually turned with a protractor.
4. Repeat five times.

If the robot consistently turns 95 degrees, the commanded wheel travel is 5 % too long and the
wheel track figure used in the calculation needs correcting by the same proportion. A 5 % turn
error produces 45 mm of lateral drift over the 520 mm pass leg, which exceeds the 30 mm of designed
clearance, so this calibration decides whether the manoeuvre clears the cone.

### C3 - feedforward fit

With the wheels lifted, hold a fixed PWM and record the steady speed at PWM = 20, 30, 40, 60 and
80. A straight-line fit through those points gives the feedforward needed to replace the fixed
`BASE_PWM = 65`.

## Known issues found by inspection

| Issue | Detail |
|---|---|
| Stage 4 regressed the detection thresholds | Stage 3C uses `OBSTACLE_DETECT_CM = 40` and `OBSTACLE_CLEAR_CM = 50`, which is correct. Stage 4 changed them to 20 and 25, while its header comment and start-up banner still state the stage 3C values. The serial log therefore reports thresholds the code does not use. **Fix: restore 40 and 50 in stage 4.** |
| Slowdown is not observable in stage 4 | With `OBSTACLE_DETECT_CM = 20` and `AVOIDANCE_TRIGGER_CM = 20`, the manoeuvre begins at the instant the obstacle is detected, so there is no interval of visibly reduced speed. Restoring the detect threshold to 40 cm gives roughly 20 cm, about 1.3 s at 60 mm/s, of visible slow driving before the bypass. |
| Start-up overshoot | 63 % in T2, caused by the oversized fixed feedforward. It will recur on every resume after each of the three stops. |
| Safety stop during the manoeuvre | While moving sideways the sensor points at whatever is beside the robot. An object within 15 cm latches the safety stop with no timeout, which would halt the robot mid-manoeuvre. |
| No echo filtering | A single spurious short reading is enough to trigger the whole manoeuvre. |
| LEDs not implemented | The brief requires mode indication. |
