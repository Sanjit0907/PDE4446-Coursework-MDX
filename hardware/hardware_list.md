# Hardware list

## Supplied in the kit

| Item | Detail | Used |
|---|---|---|
| TI-RSLK MAX chassis | Aluminium chassis, 2 x 70 mm wheels, castor wheel | Yes |
| TI-RSLK MAX chassis board | 2 x DRV8838 motor drivers, 6 x AA holder, power switch, 5 V and 3.3 V regulators | Yes |
| Gearmotors | 2 x 120:1 brushed DC, 4.5 V rated | Yes |
| Quadrature encoders | 2 x DRV5013 Hall sensors per motor | Yes |
| MSP432 LaunchPad | Replaced by the Arduino Uno | No |
| Bumper switch assemblies | Contact sensors | No - see note |
| QTRX reflectance array | Line sensor | No - not an obstacle sensor |

## Added for this project

| Item | Detail | Purpose |
|---|---|---|
| Arduino Uno | ATmega328P, 16 MHz | Main controller, replaces the LaunchPad |
| HC-SR04 ultrasonic sensor | 2-400 cm, approximately +/-15 degree beam | Obstacle detection |
| LEDs | Green, red, yellow | Operating mode indication |
| Resistors | 3 x 220 ohm | LED current limiting |
| Push button | Momentary | Manual start (planned) |
| Jumper wires, header pins | Soldered where the chassis board has unpopulated holes | Signal connections |
| 3D printed chassis extension | See `cad/chassis_extension/` | Mounting layer for the Arduino and breadboard |
| 3D printed sensor mount | See `cad/ultrasonic_sensor_mount/` | Holds the HC-SR04 forward-facing at a fixed height |

## Why these sensors

### Obstacle sensor

| Option | Beam | Range | Suitability for a 130 mm cone | Verdict |
|---|---|---|---|---|
| **HC-SR04 ultrasonic** | approximately +/-15 deg | 2-400 cm | Wide beam finds the cone reliably; sloped surfaces can deflect the echo | **Chosen** - adequate, trivial to interface, already owned |
| VL53L0X time-of-flight | approximately 25 deg | 3-200 cm | Unaffected by the cone's slope, but sensitive to ambient light and surface colour | Good upgrade |
| Sharp IR | Very narrow | 10-80 cm | Can miss an off-axis cone; non-linear output | Rejected |
| Kit bumper switches | Contact only | 0 | The cone would be struck, which fails "detect and reduce speed" | Rejected as the primary sensor; retained as a possible last-resort contact stop |
| Kit QTRX array | 1-3 cm | - | A line sensor, not an obstacle sensor | Not applicable |

### Wheel motion sensing

The kit's encoders are quadrature. Using both edges of channel A gives 720 counts per wheel
revolution, which is 0.305 mm of travel per count. Direction is measured from channel B rather
than inferred from the commanded direction, so a wheel being pushed backwards is detected
correctly.

## Electrical notes

- The motors are rated 4.5 V but run from 7-9 V. A stalled wheel can exceed the DRV8838 current
  limit, so the PWM output is capped at 150 of 255 and no wheel is ever held still against power.
- Encoder wiring is kept away from the motor wiring to limit interference.
- Battery voltage affects both the achievable speed and the PWM dead-band, so test results are
  only comparable at a similar state of charge.
