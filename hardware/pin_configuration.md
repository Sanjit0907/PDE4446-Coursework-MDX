# Arduino Uno pin configuration

As wired and used by `src/stage4_right_side_avoidance`. The TI-RSLK MAX chassis board was
designed for an MSP432 LaunchPad; the LaunchPad is not fitted, and the Arduino Uno drives the
board's signals directly.

## Motor driver (2 x DRV8838 on the chassis board)

| Chassis board signal | Arduino Uno pin | Function |
|---|---|---|
| PWML | D5 (PWM) | Left motor speed - DRV8838 EN |
| DIRL | D13 | Left motor direction - DRV8838 PH |
| PWMR | D6 (PWM) | Right motor speed |
| DIRR | D8 | Right motor direction |
| SLPL | D10 | Left driver nSLEEP |
| SLPR | D11 | Right driver nSLEEP |

Both nSLEEP lines must be driven **HIGH** to enable the drivers. With nSLEEP high and EN low the
DRV8838 brakes the motor, which is how every stop in the program is performed - the robot does not
coast.

## Encoders (quadrature, 2 x DRV5013 Hall sensors per wheel)

| Chassis board signal | Arduino Uno pin | Function |
|---|---|---|
| ELA | D2 (INT0) | Left encoder channel A - external interrupt |
| ELB | D4 | Left encoder channel B - direction |
| ERA | D3 (INT1) | Right encoder channel A - external interrupt |
| ERB | D12 | Right encoder channel B - direction |

- Channel A triggers an interrupt on **both** edges (x2 decoding); channel B is read inside the
  interrupt to determine direction.
- The chassis board pulls the Hall outputs up to 3.3 V. The sketch uses plain `INPUT`, **not** the
  Arduino's internal pull-ups, which would push 5 V into that 3.3 V rail.
- The Uno's HIGH threshold is approximately 3.0 V, leaving only ~0.3 V of margin on a 3.3 V signal.
  Missing or erratic counts would be the first thing to suspect.

## Ultrasonic sensor (HC-SR04)

| Signal | Arduino Uno pin |
|---|---|
| TRIG | D9 |
| ECHO | D7 |
| VCC | 5 V |
| GND | GND |

D10 is already used by SLPL, which is why ECHO is on D7.

## Power and ground

| Connection | Note |
|---|---|
| Common ground | The Arduino GND **must** be connected to a chassis board GND |
| Arduino supply | USB from the laptop during development |
| Motor supply | 6 x AA cells on the chassis board, through its own power switch |

The chassis board generates its own 3.3 V (TLV1117LV) and 5 V (TPS568230). The board's `3V3` pin is
an **output**. Do not connect the Arduino's 3.3 V pin to it.

Never have USB and an external 5 V feed connected to the Arduino at the same time.

## Direction configuration

Values established by test and verified on the robot:

```cpp
const bool LEFT_MOTOR_INVERTED  = false;
const bool RIGHT_MOTOR_INVERTED = false;
const bool LEFT_ENC_INVERTED    = true;   // left encoder counts down when rolling forward
const bool RIGHT_ENC_INVERTED   = false;
```

Forward corresponds to the direction pin being **LOW**. Do not change these without a specific
diagnostic reason.

## Free pins

D0 and D1 are the USB serial lines and are left alone. A0, A1 and A2 are reserved for the mode
LEDs (each through a 220 ohm resistor to the LED and then to GND).
