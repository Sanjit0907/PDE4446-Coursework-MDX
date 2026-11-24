#include <Arduino.h>

// ============================================================
// TI-RSLK MAX - STAGE 4
// Closed-loop straight driving + three stops + obstacle slowdown
// + encoder-based predetermined RIGHT-SIDE obstacle avoidance.
//
// Normal behaviour:
//   - Drive for 3 s, stop for 1 s, repeat three times.
//   - After the third stop, continue driving indefinitely.
//   - HC-SR04: <=40 cm reduces speed to 60 mm/s.
//   - At <=30 cm, begin the right-side avoidance manoeuvre.
//   - Safety stop remains active at <=15 cm during normal driving.
//
// Right-side avoidance sequence (encoder-distance controlled):
//   1. Turn right 90 degrees in place.
//   2. Move right 170 mm.
//   3. Turn left 90 degrees to face forward.
//   4. Move forward 620 mm to pass the cone.
//   5. Turn left 90 degrees.
//   6. Move left 170 mm to return to the original path.
//   7. Turn right 90 degrees to face forward.
//   8. Resume the previous driving state.
//
// Geometry used:
//   Wheel track: 148 mm (wheel centre to wheel centre)
//   Robot width: 150 mm
//   Robot length: 160 mm
//   Assumed cone footprint: 130 x 130 mm
//   Clearance margin: 30 mm
//
// IMPORTANT:
//   Encoder calibration (720 counts/rev) and wheel diameter (70 mm)
//   are inherited assumptions from Stage 3C. Verify turn and travel
//   distances at low speed in a clear area before autonomous testing.
//   The fixed forward-facing ultrasonic sensor cannot inspect the
//   side path. This is a predetermined manoeuvre, not path planning.
//   Supervise the robot and use a soft obstacle for initial tests.
//
// Serial commands:
//   w = Start/restart sequence
//   x / space = Stop and cancel
//   + / - = Increase/decrease normal target speed
//   r = Reset encoder counts
//   p = Toggle telemetry
// ============================================================

// -------------------- MOTOR PINS ----------------------------
const uint8_t PIN_PWM_L   = 5;
const uint8_t PIN_DIR_L   = 13;
const uint8_t PIN_PWM_R   = 6;
const uint8_t PIN_DIR_R   = 8;
const uint8_t PIN_SLEEP_L = 10;
const uint8_t PIN_SLEEP_R = 11;

// -------------------- ENCODER PINS --------------------------
const uint8_t PIN_ENC_L_A = 2;
const uint8_t PIN_ENC_L_B = 4;
const uint8_t PIN_ENC_R_A = 3;
const uint8_t PIN_ENC_R_B = 12;

const bool LEFT_MOTOR_INVERTED  = false;
const bool RIGHT_MOTOR_INVERTED = false;
const bool LEFT_ENC_INVERTED    = true;
const bool RIGHT_ENC_INVERTED   = false;

// -------------------- ULTRASONIC PINS -----------------------
const uint8_t TRIG_PIN = 9;
const uint8_t ECHO_PIN = 7;

// -------------------- ROBOT / WHEEL GEOMETRY ----------------
const float WHEEL_DIAMETER_MM = 70.0f;
const float COUNTS_PER_REV = 720.0f;  // inherited Stage 3C assumption
const float TRACK_WIDTH_MM = 148.0f;
const float ROBOT_WIDTH_MM = 150.0f;
const float ROBOT_LENGTH_MM = 160.0f;
const float CONE_WIDTH_MM = 130.0f;
const float CONE_LENGTH_MM = 130.0f;
const float CLEARANCE_MARGIN_MM = 30.0f;

const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

// Lateral centre offset = half robot width + half cone width + margin.
const float LATERAL_OFFSET_MM =
  (ROBOT_WIDTH_MM * 0.5f) + (CONE_WIDTH_MM * 0.5f) + CLEARANCE_MARGIN_MM;

// At the avoidance trigger, the sensor is nominally this far from
// the front face of the cone. Add cone length, robot length and margin
// so the robot's rear clears the cone before returning to the path.
const float AVOIDANCE_TRIGGER_CM = 20.0f;
const float AVOIDANCE_PASS_DISTANCE_MM =
  (AVOIDANCE_TRIGGER_CM * 10.0f) + CONE_LENGTH_MM +
  ROBOT_LENGTH_MM + CLEARANCE_MARGIN_MM;

// -------------------- PI CONTROLLER -------------------------
float KP = 0.20f;
float KI = 0.50f;
float targetSpeed = 120.0f;
const float OBSTACLE_SPEED = 60.0f;
const float AVOIDANCE_SPEED = 60.0f;
const float BASE_PWM = 65.0f;
const int PWM_MIN = 0;
const int PWM_MAX = 150;
const float INTEGRAL_LIMIT = 60.0f;

// -------------------- OBSTACLE SETTINGS ---------------------
const float OBSTACLE_DETECT_CM = 20.0f;
const float OBSTACLE_CLEAR_CM = 25.0f;
const float SAFETY_STOP_CM = 15.0f;
const float SAFETY_RESUME_CM = 20.0f;
const uint32_t SENSOR_INTERVAL_MS = 100;
const uint32_t ECHO_TIMEOUT_US = 25000;

float distanceCm = -1.0f;
bool obstacleDetected = false;
bool safetyStopActive = false;
bool safetyStopPausedDriveTimer = false;
uint32_t safetyStopStartMs = 0;
uint32_t lastSensorReadMs = 0;

// -------------------- SEQUENCE TIMING -----------------------
const uint32_t DRIVE_DURATION_MS = 3000;
const uint32_t STOP_DURATION_MS = 1000;
const uint8_t TOTAL_STOPS = 3;

// -------------------- CONTROL TIMING ------------------------
const uint32_t CONTROL_INTERVAL_MS = 100;
const uint32_t TELEMETRY_INTERVAL_MS = 200;

// -------------------- ENCODER COUNTS ------------------------
volatile long ticksL = 0;
volatile long ticksR = 0;

void isrLeft() {
  int8_t d = (digitalRead(PIN_ENC_L_A) != digitalRead(PIN_ENC_L_B)) ? 1 : -1;
  ticksL += LEFT_ENC_INVERTED ? -d : d;
}

void isrRight() {
  int8_t d = (digitalRead(PIN_ENC_R_A) != digitalRead(PIN_ENC_R_B)) ? 1 : -1;
  ticksR += RIGHT_ENC_INVERTED ? -d : d;
}

long readTicks(volatile long &counter) {
  noInterrupts();
  long value = counter;
  interrupts();
  return value;
}

// -------------------- MOTOR OUTPUT --------------------------
int pwmL = 0;
int pwmR = 0;

void driveMotor(uint8_t pwmPin, uint8_t dirPin, bool inverted, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm == 0) {
    analogWrite(pwmPin, 0);
    return;
  }
  bool forward = (pwm > 0) != inverted;
  digitalWrite(dirPin, forward ? LOW : HIGH);
  analogWrite(pwmPin, abs(pwm));
}

void stopMotors() {
  pwmL = 0;
  pwmR = 0;
  driveMotor(PIN_PWM_L, PIN_DIR_L, LEFT_MOTOR_INVERTED, 0);
  driveMotor(PIN_PWM_R, PIN_DIR_R, RIGHT_MOTOR_INVERTED, 0);
}

// -------------------- PI CONTROLLER -------------------------
float integralL = 0.0f;
float integralR = 0.0f;
float velocityL = 0.0f;
float velocityR = 0.0f;
long previousTicksL = 0;
long previousTicksR = 0;

int calculatePI(float target, float measured, float &integral, float dt) {
  float error = target - measured;
  float candidateIntegral = integral + KI * error * dt;
  candidateIntegral = constrain(candidateIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float output = BASE_PWM + KP * error + candidateIntegral;
  float limitedOutput = constrain(output, (float)PWM_MIN, (float)PWM_MAX);
  if (output == limitedOutput ||
      (output > PWM_MAX && error < 0) ||
      (output < PWM_MIN && error > 0)) {
    integral = candidateIntegral;
  }
  return (int)constrain(limitedOutput, (float)PWM_MIN, (float)PWM_MAX);
}

void resetDriveController() {
  integralL = 0.0f;
  integralR = 0.0f;
  pwmL = 0;
  pwmR = 0;
  previousTicksL = readTicks(ticksL);
  previousTicksR = readTicks(ticksR);
  velocityL = 0.0f;
  velocityR = 0.0f;
}

// -------------------- ULTRASONIC MEASUREMENT ----------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;
  return (duration * 0.0343f) / 2.0f;
}

// -------------------- ROBOT STATES --------------------------
enum RobotState { IDLE, DRIVING, STOPPED, FINISHED, AVOIDING };
RobotState robotState = IDLE;
RobotState stateToResumeAfterAvoidance = DRIVING;
uint8_t completedStops = 0;
uint32_t stateStartMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;
bool telemetryOn = true;

// -------------------- AVOIDANCE MOTION ----------------------
enum AvoidStep {
  TURN_RIGHT_OUT,
  SHIFT_RIGHT,
  TURN_LEFT_TO_FORWARD,
  PASS_FORWARD,
  TURN_LEFT_TO_RETURN,
  SHIFT_LEFT,
  TURN_RIGHT_TO_FORWARD,
  AVOIDANCE_COMPLETE
};

AvoidStep avoidStep = TURN_RIGHT_OUT;
long moveStartTicksL = 0;
long moveStartTicksR = 0;
long moveTargetTicks = 0;
int8_t moveSignL = 1;
int8_t moveSignR = 1;
uint32_t avoidanceStartMs = 0;

long mmToTicks(float distanceMm) {
  return (long)(distanceMm / MM_PER_TICK + 0.5f);
}

const float TURN_90_WHEEL_DISTANCE_MM = (PI * TRACK_WIDTH_MM) / 4.0f;

void startEncoderMove(int8_t signL, int8_t signR, float distanceMm) {
  stopMotors();
  integralL = 0.0f;
  integralR = 0.0f;
  moveSignL = signL;
  moveSignR = signR;
  moveTargetTicks = mmToTicks(distanceMm);
  moveStartTicksL = readTicks(ticksL);
  moveStartTicksR = readTicks(ticksR);
  previousTicksL = moveStartTicksL;
  previousTicksR = moveStartTicksR;
  velocityL = 0.0f;
  velocityR = 0.0f;
  lastControlMs = millis();
}

void startCurrentAvoidStep();
void beginAvoidance();

void startCurrentAvoidStep() {
  switch (avoidStep) {
    case TURN_RIGHT_OUT:
      Serial.println(F("# Avoidance: turn RIGHT 90 deg"));
      startEncoderMove(+1, -1, TURN_90_WHEEL_DISTANCE_MM);
      break;
    case SHIFT_RIGHT:
      Serial.println(F("# Avoidance: shift RIGHT"));
      startEncoderMove(+1, +1, LATERAL_OFFSET_MM);
      break;
    case TURN_LEFT_TO_FORWARD:
      Serial.println(F("# Avoidance: turn LEFT 90 deg"));
      startEncoderMove(-1, +1, TURN_90_WHEEL_DISTANCE_MM);
      break;
    case PASS_FORWARD:
      Serial.println(F("# Avoidance: pass obstacle"));
      startEncoderMove(+1, +1, AVOIDANCE_PASS_DISTANCE_MM);
      break;
    case TURN_LEFT_TO_RETURN:
      Serial.println(F("# Avoidance: turn LEFT 90 deg"));
      startEncoderMove(-1, +1, TURN_90_WHEEL_DISTANCE_MM);
      break;
    case SHIFT_LEFT:
      Serial.println(F("# Avoidance: return LEFT to original path"));
      startEncoderMove(-1, -1, LATERAL_OFFSET_MM);
      break;
    case TURN_RIGHT_TO_FORWARD:
      Serial.println(F("# Avoidance: turn RIGHT 90 deg to face forward"));
      startEncoderMove(+1, -1, TURN_90_WHEEL_DISTANCE_MM);
      break;
    case AVOIDANCE_COMPLETE:
      break;
  }
}

void beginAvoidance() {
  if (robotState == AVOIDING || safetyStopActive) return;

  stateToResumeAfterAvoidance = robotState;
  avoidanceStartMs = millis();
  robotState = AVOIDING;
  avoidStep = TURN_RIGHT_OUT;
  stopMotors();
  resetDriveController();

  Serial.println();
  Serial.println(F("# ======================================="));
  Serial.println(F("# STAGE 4: RIGHT-SIDE AVOIDANCE STARTED"));
  Serial.print(F("# Wheel track: ")); Serial.print(TRACK_WIDTH_MM); Serial.println(F(" mm"));
  Serial.print(F("# Lateral offset: ")); Serial.print(LATERAL_OFFSET_MM); Serial.println(F(" mm"));
  Serial.print(F("# Forward pass distance: ")); Serial.print(AVOIDANCE_PASS_DISTANCE_MM); Serial.println(F(" mm"));
  Serial.println(F("# Keep the test area clear and supervise the robot."));
  Serial.println(F("# ======================================="));

  startCurrentAvoidStep();
}

void advanceAvoidanceStep() {
  stopMotors();
  integralL = 0.0f;
  integralR = 0.0f;

  if (avoidStep == TURN_RIGHT_TO_FORWARD) {
    avoidStep = AVOIDANCE_COMPLETE;
  } else {
    avoidStep = (AvoidStep)((int)avoidStep + 1);
  }

  if (avoidStep == AVOIDANCE_COMPLETE) {
    uint32_t now = millis();
    uint32_t elapsed = now - avoidanceStartMs;

    if (stateToResumeAfterAvoidance == DRIVING) {
      // Do not count the avoidance manoeuvre as part of the timed
      // three-second straight-driving interval.
      stateStartMs += elapsed;
    }

    robotState = stateToResumeAfterAvoidance;
    obstacleDetected = false;
    safetyStopActive = false;
    safetyStopPausedDriveTimer = false;
    resetDriveController();

    Serial.println();
    Serial.println(F("# RIGHT-SIDE AVOIDANCE COMPLETE"));
    Serial.println(F("# Resuming previous driving mode."));
  } else {
    startCurrentAvoidStep();
  }
}

void updateAvoidanceControl(uint32_t now) {
  if (robotState != AVOIDING) return;
  // Keep the close-range safety stop active during the manoeuvre too.
  // The fixed sensor is facing whichever direction the robot currently faces.
  if (safetyStopActive) {
    stopMotors();
    return;
  }
  if (now - lastControlMs < CONTROL_INTERVAL_MS) return;

  float dt = (now - lastControlMs) / 1000.0f;
  lastControlMs = now;

  long currentL = readTicks(ticksL);
  long currentR = readTicks(ticksR);

  long progressL = (currentL - moveStartTicksL) * moveSignL;
  long progressR = (currentR - moveStartTicksR) * moveSignR;

  long deltaL = currentL - previousTicksL;
  long deltaR = currentR - previousTicksR;
  previousTicksL = currentL;
  previousTicksR = currentR;

  velocityL = (deltaL * MM_PER_TICK) / dt;
  velocityR = (deltaR * MM_PER_TICK) / dt;

  bool leftDone = progressL >= moveTargetTicks;
  bool rightDone = progressR >= moveTargetTicks;

  if (leftDone && rightDone) {
    advanceAvoidanceStep();
    return;
  }

  int magnitudeL = leftDone ? 0 : calculatePI(AVOIDANCE_SPEED, fabs(velocityL), integralL, dt);
  int magnitudeR = rightDone ? 0 : calculatePI(AVOIDANCE_SPEED, fabs(velocityR), integralR, dt);

  pwmL = moveSignL * magnitudeL;
  pwmR = moveSignR * magnitudeR;

  driveMotor(PIN_PWM_L, PIN_DIR_L, LEFT_MOTOR_INVERTED, pwmL);
  driveMotor(PIN_PWM_R, PIN_DIR_R, RIGHT_MOTOR_INVERTED, pwmR);
}

// -------------------- OBSTACLE DETECTION --------------------
void updateObstacleDetection() {
  uint32_t now = millis();
  if (now - lastSensorReadMs < SENSOR_INTERVAL_MS) return;
  lastSensorReadMs = now;
  distanceCm = readDistanceCm();

  if (robotState == AVOIDING) {
    // During avoidance, do not re-trigger slowdown or start another
    // manoeuvre. Still honour the close-range safety stop. A missing
    // echo while already safety-stopped does not clear the stop.
    if (distanceCm >= 0.0f) {
      if (!safetyStopActive && distanceCm <= SAFETY_STOP_CM) {
        safetyStopActive = true;
        stopMotors();
        Serial.println(F("# AVOIDANCE SAFETY STOP: measured distance <=15 cm"));
      } else if (safetyStopActive && distanceCm >= SAFETY_RESUME_CM) {
        safetyStopActive = false;
        integralL = 0.0f;
        integralR = 0.0f;
        previousTicksL = readTicks(ticksL);
        previousTicksR = readTicks(ticksR);
        lastControlMs = now;
        Serial.println(F("# AVOIDANCE SAFETY STOP CLEARED"));
      }
    }
    return;
  }

  if (distanceCm >= 0.0f) {
    if (!safetyStopActive && distanceCm <= SAFETY_STOP_CM) {
      safetyStopActive = true;
      safetyStopStartMs = now;
      safetyStopPausedDriveTimer = (robotState == DRIVING);
      stopMotors();
      integralL = 0.0f;
      integralR = 0.0f;
      velocityL = 0.0f;
      velocityR = 0.0f;
      previousTicksL = readTicks(ticksL);
      previousTicksR = readTicks(ticksR);
      Serial.println(F("# SAFETY STOP: obstacle <=15 cm; waiting for >=20 cm"));
    }
    else if (safetyStopActive && distanceCm >= SAFETY_RESUME_CM) {
      safetyStopActive = false;
      if (safetyStopPausedDriveTimer && robotState == DRIVING) {
        stateStartMs += (now - safetyStopStartMs);
      }
      safetyStopPausedDriveTimer = false;
      resetDriveController();
      lastControlMs = now;
      Serial.println(F("# SAFETY STOP CLEARED"));
    }

    if (!obstacleDetected && distanceCm <= OBSTACLE_DETECT_CM) {
      obstacleDetected = true;
      Serial.println(F("# OBSTACLE DETECTED: reduced speed target is 60 mm/s"));
    }
    else if (obstacleDetected && distanceCm >= OBSTACLE_CLEAR_CM) {
      obstacleDetected = false;
      Serial.println(F("# PATH CLEAR: normal speed restored"));
    }

    // Start the bypass at 30 cm, after the 40 cm slowdown threshold
    // and before the 15 cm emergency-stop threshold.
    if (!safetyStopActive && distanceCm <= AVOIDANCE_TRIGGER_CM &&
        (robotState == DRIVING || robotState == FINISHED)) {
      beginAvoidance();
    }
  }
}

float getActiveTargetSpeed() {
  if (obstacleDetected) return min(targetSpeed, OBSTACLE_SPEED);
  return targetSpeed;
}

// -------------------- SEQUENCE CONTROL -----------------------
void startSequence() {
  stopMotors();
  completedStops = 0;
  safetyStopActive = false;
  safetyStopPausedDriveTimer = false;
  obstacleDetected = false;
  robotState = DRIVING;
  resetDriveController();
  stateStartMs = millis();
  Serial.println(F("# Stage 4 sequence started"));
  Serial.println(F("# Three 3-second drives / 1-second stops, then continuous drive"));
  Serial.println(F("# Right-side avoidance triggers at 30 cm"));
}

void cancelSequence() {
  stopMotors();
  integralL = 0.0f;
  integralR = 0.0f;
  robotState = IDLE;
  completedStops = 0;
  safetyStopActive = false;
  safetyStopPausedDriveTimer = false;
  obstacleDetected = false;
  velocityL = 0.0f;
  velocityR = 0.0f;
  Serial.println(F("# Sequence cancelled - robot stopped"));
}

void updateSequence(uint32_t now) {
  if (robotState == DRIVING && !safetyStopActive &&
      now - stateStartMs >= DRIVE_DURATION_MS) {
    stopMotors();
    integralL = 0.0f;
    integralR = 0.0f;
    velocityL = 0.0f;
    velocityR = 0.0f;
    completedStops++;
    robotState = STOPPED;
    stateStartMs = now;
    Serial.print(F("# Stop "));
    Serial.print(completedStops);
    Serial.println(F(" of 3 - waiting 1 second"));
  }
  else if (robotState == STOPPED && now - stateStartMs >= STOP_DURATION_MS) {
    resetDriveController();
    stateStartMs = now;
    if (completedStops < TOTAL_STOPS) {
      robotState = DRIVING;
      Serial.println(F("# Resuming forward movement"));
    } else {
      robotState = FINISHED;
      Serial.println(F("# THREE-STOP SEQUENCE COMPLETE - continuing straight"));
    }
  }
}

// -------------------- SERIAL COMMANDS ------------------------
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'w': case 'W': startSequence(); break;
      case 'x': case 'X': case ' ': cancelSequence(); break;
      case '+': case '=':
        targetSpeed = min(250.0f, targetSpeed + 10.0f);
        Serial.print(F("# Normal target speed: ")); Serial.println(targetSpeed);
        break;
      case '-': case '_':
        targetSpeed = max(40.0f, targetSpeed - 10.0f);
        Serial.print(F("# Normal target speed: ")); Serial.println(targetSpeed);
        break;
      case 'r': case 'R':
        noInterrupts(); ticksL = 0; ticksR = 0; interrupts();
        previousTicksL = 0; previousTicksR = 0;
        Serial.println(F("# Encoder counts reset"));
        break;
      case 'p': case 'P':
        telemetryOn = !telemetryOn;
        Serial.print(F("# Telemetry ")); Serial.println(telemetryOn ? F("ON") : F("OFF"));
        break;
    }
  }
}

// -------------------- SETUP ---------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM_L, OUTPUT); pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_PWM_R, OUTPUT); pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_SLEEP_L, OUTPUT); pinMode(PIN_SLEEP_R, OUTPUT);
  analogWrite(PIN_PWM_L, 0); analogWrite(PIN_PWM_R, 0);
  digitalWrite(PIN_SLEEP_L, HIGH); digitalWrite(PIN_SLEEP_R, HIGH);

  pinMode(PIN_ENC_L_A, INPUT); pinMode(PIN_ENC_L_B, INPUT);
  pinMode(PIN_ENC_R_A, INPUT); pinMode(PIN_ENC_R_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRight, CHANGE);

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  lastControlMs = millis();
  lastTelemetryMs = millis();
  lastSensorReadMs = millis();
  resetDriveController();

  Serial.println();
  Serial.println(F("# TI-RSLK MAX - STAGE 4 RIGHT-SIDE AVOIDANCE"));
  Serial.println(F("# Motor / encoder / ultrasonic pins retained from Stage 3C"));
  Serial.println(F("# Wheel track: 148 mm | chassis: 150 x 160 mm"));
  Serial.print(F("# Theoretical 90-degree wheel travel: "));
  Serial.print(TURN_90_WHEEL_DISTANCE_MM, 2); Serial.println(F(" mm"));
  Serial.print(F("# Theoretical ticks per 90-degree turn: "));
  Serial.println(mmToTicks(TURN_90_WHEEL_DISTANCE_MM));
  Serial.print(F("# Lateral offset: ")); Serial.print(LATERAL_OFFSET_MM);
  Serial.println(F(" mm"));
  Serial.print(F("# Forward pass distance: ")); Serial.print(AVOIDANCE_PASS_DISTANCE_MM);
  Serial.println(F(" mm"));
  Serial.println(F("# Slow at 40 cm; avoidance starts at 30 cm; safety stop at 15 cm"));
  Serial.println(F("# Commands: w=start, x=stop, +/-=speed, r=reset encoders, p=telemetry"));
  Serial.println(F("# Send w to begin"));
}

// -------------------- MAIN LOOP -----------------------------
void loop() {
  handleSerial();
  uint32_t now = millis();

  updateObstacleDetection();

  if (robotState == AVOIDING) {
    updateAvoidanceControl(millis());
  } else {
    updateSequence(now);

    if ((robotState == DRIVING || robotState == FINISHED) && !safetyStopActive) {
      now = millis();
      if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
        float dt = (now - lastControlMs) / 1000.0f;
        lastControlMs = now;
        long currentL = readTicks(ticksL);
        long currentR = readTicks(ticksR);
        long deltaL = currentL - previousTicksL;
        long deltaR = currentR - previousTicksR;
        previousTicksL = currentL;
        previousTicksR = currentR;
        velocityL = (deltaL * MM_PER_TICK) / dt;
        velocityR = (deltaR * MM_PER_TICK) / dt;
        float activeTargetSpeed = getActiveTargetSpeed();
        pwmL = calculatePI(activeTargetSpeed, velocityL, integralL, dt);
        pwmR = calculatePI(activeTargetSpeed, velocityR, integralR, dt);
        driveMotor(PIN_PWM_L, PIN_DIR_L, LEFT_MOTOR_INVERTED, pwmL);
        driveMotor(PIN_PWM_R, PIN_DIR_R, RIGHT_MOTOR_INVERTED, pwmR);
      }
    }
  }

  now = millis();
  if (telemetryOn && now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    Serial.print(F("STATE,"));
    switch (robotState) {
      case IDLE: Serial.print(F("IDLE")); break;
      case DRIVING: Serial.print(F("DRIVING")); break;
      case STOPPED: Serial.print(F("STOPPED")); break;
      case FINISHED: Serial.print(F("CONTINUOUS")); break;
      case AVOIDING: Serial.print(F("AVOIDING")); break;
    }
    Serial.print(F(",DIST_CM,"));
    if (distanceCm < 0.0f) Serial.print(F("NO_ECHO"));
    else Serial.print(distanceCm, 1);
    Serial.print(F(",MODE,"));
    if (robotState == AVOIDING) {
      switch (avoidStep) {
        case TURN_RIGHT_OUT: Serial.print(F("TURN_RIGHT_OUT")); break;
        case SHIFT_RIGHT: Serial.print(F("SHIFT_RIGHT")); break;
        case TURN_LEFT_TO_FORWARD: Serial.print(F("TURN_LEFT_TO_FORWARD")); break;
        case PASS_FORWARD: Serial.print(F("PASS_FORWARD")); break;
        case TURN_LEFT_TO_RETURN: Serial.print(F("TURN_LEFT_TO_RETURN")); break;
        case SHIFT_LEFT: Serial.print(F("SHIFT_LEFT")); break;
        case TURN_RIGHT_TO_FORWARD: Serial.print(F("TURN_RIGHT_TO_FORWARD")); break;
        case AVOIDANCE_COMPLETE: Serial.print(F("AVOIDANCE_COMPLETE")); break;
      }
    } else if (safetyStopActive) Serial.print(F("SAFETY_STOP"));
    else if (obstacleDetected) Serial.print(F("SLOW"));
    else Serial.print(F("NORMAL"));

    Serial.print(F(",VL,")); Serial.print(velocityL, 1);
    Serial.print(F(",VR,")); Serial.print(velocityR, 1);
    Serial.print(F(",PWM_L,")); Serial.print(pwmL);
    Serial.print(F(",PWM_R,")); Serial.print(pwmR);
    Serial.print(F(",TICKS_L,")); Serial.print(readTicks(ticksL));
    Serial.print(F(",TICKS_R,")); Serial.println(readTicks(ticksR));
  }
}
