# PDE4446 Motion Control - Coursework 1

Autonomous motion control for a two-wheel differential-drive mobile robot, built on the
**TI-RSLK MAX** chassis with an **Arduino Uno** replacing the MSP432 LaunchPad.

## Team

| Name | Student ID | Main responsibility |
|---|---|---|
| Sanjit Sriprasanna | M01142490 | Firmware, control loops, obstacle avoidance |
| Havinesh Kumar | M01152797 | Hardware integration, CAD and 3D printing, test data |

Middlesex University Dubai - MSc Robotics

## What the robot does

1. Drives in a straight line along a designated route (~4.4 m) using encoder-based closed-loop
   PI speed control on each wheel.
2. Stops three times, each stop after three seconds of driving, holding each stop for one second.
3. Detects a randomly placed obstacle with a forward-facing HC-SR04 ultrasonic sensor and
   reduces speed.
4. Navigates around the obstacle using a predetermined encoder-measured right-side manoeuvre.
5. Returns to the original path and to normal travelling speed.
6. Indicates the current operating mode on LEDs.

## Repository layout

| Path | Contents |
|---|---|
| `src/` | Arduino sketches, one folder per sketch (required by the Arduino IDE) |
| `hardware/` | Pin map, wiring diagram, component list |
| `cad/` | 3D printed parts: chassis extension layer and ultrasonic sensor mount |
| `docs/` | Development timeline, system design and justification, test results |
| `tests/` | Raw measurement data in CSV form |

## Sketches

Each sketch is a development stage, kept so the progression is visible and so any stage can be
re-run on its own for testing. Folder names match the `.ino` filenames, as the Arduino IDE requires.

| Stage | Sketch | Purpose |
|---|---|---|
| 0 | `src/stage0_teleop_test/` | Bring-up: drive each motor and read each encoder from keyboard commands. Open loop |
| 1 | `src/stage1_straight_line_pid/` | Encoder-based closed-loop PI speed control on each wheel, straight-line driving |
| 2 | `src/stage2_three_second_stops/` | Adds the timed sequence: drive 3 s, stop 1 s, three times |
| 3A | `src/stage3a_ultrasonic_test/` | HC-SR04 distance measurement on its own, no motor control |
| 3C | `src/stage3c_obstacle_safety_stop/` | Adds obstacle detection, speed reduction to 60 mm/s at 40 cm, and a safety stop at 15 cm |
| 4 | `src/stage4_right_side_avoidance/` | **Current main program.** Adds the predetermined right-side bypass manoeuvre |

The motor and encoder bring-up tests are both covered by stage 0; there is no separate
`motor_test` or `encoder_test`. A combined `final_robot` sketch has not been written yet - stage 4
is the working program.

## How to run

1. Open the sketch in the Arduino IDE and select **Arduino Uno** and the correct COM port.
2. Upload.
3. Open Serial Monitor at **115200 baud**, line ending **No line ending**.
4. Place the robot at the start of the route and send `w` to begin.

| Key | Action |
|---|---|
| `w` | Start the sequence |
| `x` or space | Stop and cancel |
| `+` / `-` | Increase / decrease the normal target speed |
| `r` | Reset the encoder counts |
| `p` | Toggle telemetry |

## Current status

| Item | Status |
|---|---|
| Motors and encoders | Working |
| Straight-line PI control | Working, tested |
| Three timed stops | Working |
| Ultrasonic detection and speed reduction | Working in stage 3C; thresholds need restoring in stage 4 - see Limitations |
| Right-side avoidance manoeuvre | Implemented |
| Mode LEDs | To do |
| Encoder and turn calibration | To do - see Limitations |

## Limitations

1. **Encoder calibration is not verified.** `COUNTS_PER_REV = 720` and a 70 mm wheel diameter are
   derived values, not measured ones, so every distance and speed reported in millimetres is
   unverified in absolute terms. Left-versus-right comparisons remain valid.
2. **The ultrasonic sensor is fixed and forward-facing.** It cannot scan to find which side is
   clear, so the avoidance direction is predetermined (right) rather than planned.
3. **The avoidance manoeuvre assumes the obstacle's size.** Its lateral offset and pass distance
   are computed from the coursework cone's dimensions (130 x 130 mm). A larger obstacle would not
   be cleared.
4. **No heading feedback.** Matching the two wheel speeds does not hold a heading. An error in the
   relative wheel size, or wheel slip during a pivot, accumulates with no sensor to correct it.
5. **USB cable drag affects straight-line tests.** See `docs/testing_and_results.md`, test T3.
6. **Stage 4 regressed the detection thresholds.** Stage 3C slows at 40 cm and clears at 50 cm.
   Stage 4 sets both the slowdown and the avoidance trigger to 20 cm, so the robot begins the
   manoeuvre at the moment it detects the obstacle and never drives at visibly reduced speed. The
   stage 4 header comment still describes the stage 3C values. To be corrected.
