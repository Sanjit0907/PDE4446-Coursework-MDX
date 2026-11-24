# System design and justification

Covers the rubric criterion *System design and component selection*. Each choice is given with the
alternatives that were rejected and the numbers behind it.

## 1. Interpreting the brief

| Brief wording | Interpretation used | Consequence of being wrong |
|---|---|---|
| "Stop three times every three seconds along the route" | **Time-based**: drive 3 s, stop, repeat. This is the literal reading. | At 120 mm/s the three stops fall in the first ~1.1 m of a 4.4 m route. If even spatial distribution is required instead, only the cruise speed or the leg rule needs to change. |
| "Wait for 1 second on each stop" | A 1000 ms dwell with the wheels braked | - |
| "Detect randomly placed obstacle, reduce the speed, and navigate around it" | Non-contact detection, then a reduced target speed, then the bypass | Contact sensing would fail "detect ... reduce the speed" |
| "Return to the original path and back to its normal travelling speed" | Rejoin the same line and restore the cruise target | - |
| Route length | Approximately 4.4 m | - |
| Obstacle | Plastic cone, approximately 130 x 130 x 165 mm, placed on the robot's path | A wider obstacle would not be cleared - see Limitations |

## 2. Architecture

```
   HC-SR04  ──► distance ──┐
                            │
   encoders ──► tick ISRs ──┼──► state machine ──► per-wheel speed targets
                 (x2 quad)  │     IDLE / DRIVING / STOPPED             │
                            │     CONTINUOUS / AVOIDING               │
   serial ────► commands ───┘                                          ▼
                                                        per-wheel PI controller
                                                                       │
                                                                       ▼
                                                     PWM + DIR ──► DRV8838 ──► motors
                                                                       │
                                                      LEDs ◄── mode ───┘
```

| Subsystem | Choice | Reason |
|---|---|---|
| Controller | Arduino Uno | Its two external interrupts, D2 and D3, are exactly what the two encoder A channels need. 16 MHz is ample for a 10 Hz control loop. |
| Motor driver | DRV8838 on the chassis board | Already fitted. PH/EN interface; EN low with nSLEEP high gives a true brake rather than a coast. |
| Wheel sensing | Quadrature encoders, x2 decoding | 0.305 mm per count, and direction is measured rather than assumed. |
| Speed control | Independent PI loop per wheel | Corrects for motor mismatch, load and battery sag without needing matched motors. |
| Obstacle sensing | Single fixed forward-facing HC-SR04 | Adequate for a cone on the path; a scanning servo was explicitly ruled out. |
| Behaviour | Non-blocking `millis()` state machine | A `delay()` would freeze the PI loop, the encoder reads and the serial interface. |

## 3. Calculations

**Distance per encoder count**

```
Given:   wheel diameter D = 70 mm, counts per revolution N = 720
Formula: d = pi * D / N
         d = 3.1416 * 70 / 720
Answer:  d = 0.3054 mm per count
```
At a 120 mm/s cruise that is 393 counts/s per wheel, so 786 interrupts per second for both wheels.
Each interrupt service routine takes roughly 5-10 us, which is under 1 % of the processor.

**Counts per revolution (derived, not yet measured)**

```
Given:   12 quadrature counts per motor-shaft revolution, 120:1 gearbox
Formula: N_quad = 12 * 120 = 1440 counts per wheel revolution (all edges of A and B)
         Channel A, both edges only = N_quad / 2
Answer:  N = 720 counts per wheel revolution
```
This must be confirmed by measurement - see Limitations and test C1.

**Wheel travel for a 90 degree pivot**

```
Given:   wheel track T = 148 mm (wheel centre to wheel centre)
Formula: s = pi * T / 4
         s = 3.1416 * 148 / 4
Answer:  s = 116.2 mm per wheel = 381 encoder counts
```
Each wheel travels this distance, in opposite directions, so the robot turns about the midpoint of
its axle.

**Lateral offset for the bypass**

```
Given:   robot width 150 mm, cone width 130 mm, clearance margin 30 mm
Formula: L = robot_width/2 + cone_width/2 + margin
         L = 75 + 65 + 30
Answer:  L = 170 mm
```
Remaining clearance between the robot's inner edge and the cone is 170 - 75 - 65 = **30 mm**.

**Forward distance to pass the cone**

```
Given:   trigger distance 200 mm (sensor face to cone front face),
         cone depth 130 mm, robot length 160 mm, margin 30 mm
Formula: P = trigger + cone_depth + robot_length + margin
         P = 200 + 130 + 160 + 30
Answer:  P = 520 mm
```
This is correct only if the ultrasonic sensor sits at the robot's front face. That must be measured.

**Sensitivity of the manoeuvre to turn error**

```
Given:   a heading error e after a turn, followed by a pass leg of 520 mm
Formula: lateral drift = 520 * sin(e)
         e = 5 deg  ->  520 * 0.0872 = 45 mm
Answer:  45 mm of drift from a 5 degree turn error
```
This **exceeds the 30 mm clearance above**, which is why turn calibration is the single most
important outstanding task. The present uncertainty in the wheel track, between 141 mm and 148 mm,
is itself 5 %.

**Ultrasonic timing**

```
Given:   speed of sound c = 0.0343 cm/us, echo time t
Formula: distance = t * c / 2
         t = 25000 us (the current timeout)  ->  429 cm
```
The timeout sets the maximum range. It also sets how long `pulseIn` can block the main loop, which
at 25000 us is 25 ms - a quarter of the 100 ms control interval. Reducing it to 8000 us gives
1.4 m of range, which is far more than the 40 cm needed, and caps the block at 8 ms.

## 4. Control design

Each wheel runs its own PI controller on measured speed:

```
pwm = feedforward + Kp * error + integral(Ki * error)
```

- **Feedforward** supplies most of the output so the integrator is not fighting the motor's
  dead-band. Measurements show approximately PWM 29 holds 120 mm/s.
- **Anti-windup** by conditional integration: the integral only accumulates when the output is not
  saturated, or when the error would bring it out of saturation.
- **Current gains**: `Kp = 0.20`, `Ki = 0.50`, integral clamped to +/-60, PWM clamped to 0-150.
- Equal wheel speeds are necessary for straight travel but not sufficient. Wheel diameter
  differences, alignment, floor conditions and cable forces all still affect the path, because
  nothing in the system measures heading.

## 5. Obstacle response

| Measured distance | Behaviour |
|---|---|
| Above the detect threshold | Normal target speed |
| At or below the detect threshold | Target speed reduced to 60 mm/s |
| At or below the avoidance trigger | Right-side bypass manoeuvre begins |
| At or below 15 cm during normal driving | Safety stop until the path clears |

The bypass is eight steps, each ended by an encoder count rather than a timer:

```
   1. pivot right 90 deg          5. pivot left 90 deg
   2. move across 170 mm          6. move across 170 mm
   3. pivot left 90 deg           7. pivot right 90 deg
   4. drive forward 520 mm        8. resume the previous driving state
```

Encoder counts rather than timed turns: a timed turn assumes a known wheel speed, and wheel speed
falls by roughly 10 % between a fresh and a half-drained battery pack. That turns a 90 degree
command into 81 degrees, which is 78 mm of drift over the pass leg - enough to strike the cone, and
it would appear only in the final demonstration when the batteries are at their lowest.

The three-second driving timer is paused for the duration of the manoeuvre, so the bypass does not
consume part of a driving leg.

## 6. Operating modes and LED indication

| Mode | Green | Red | Yellow |
|---|---|---|---|
| Idle | - | - | - |
| Normal driving | On | - | - |
| Stopped (scheduled or safety) | - | On | - |
| Obstacle detected, reduced speed | On | - | On |
| Avoidance in progress | - | - | On |

LED wiring is planned on A0, A1 and A2. **Not yet implemented in the sketch.**

## 7. Alternatives considered for the avoidance

| Method | Hardware | Handles unknown obstacle? | Repeatability | Verdict |
|---|---|---|---|---|
| Timed rectangle | None | No | Poor - battery dependent | Rejected |
| **Encoder-measured rectangle** | None | No | Adequate with calibration | **Chosen** |
| Rotate-and-probe with the single sensor | None | Yes | Adequate | Rejected - many stop-and-measure steps make the demonstration slow |
| Second, side-facing ultrasonic | +1 sensor | Yes | Adequate | Strong upgrade; not fitted |
| Gyroscope-assisted turns | +1 MPU-6050 | Fixes turn error, not obstacle size | Good | Strong upgrade; not fitted |
| Bumper switches | In the kit | Yes | Poor | Rejected - contact fails the brief |
| Servo-scanned ultrasonic | +1 servo | Yes | Adequate | Ruled out by the project constraints |

## 8. Future work, in order of value

1. Calibrate `COUNTS_PER_REV` and the wheel track by measurement, which is the dominant error.
2. Add a side-facing ultrasonic so "have I passed the obstacle?" becomes a measurement rather than
   an assumption, making the manoeuvre independent of obstacle size.
3. Add an MPU-6050 so turns close the loop on measured angle instead of counted wheel rotation.
4. Replace the pivots with arcs: arcs scrub the wheels far less than a pivot in place, so they lose
   less to slip, and the motion is smoother.
5. Use the kit's bumper switches as a last-resort contact stop.
