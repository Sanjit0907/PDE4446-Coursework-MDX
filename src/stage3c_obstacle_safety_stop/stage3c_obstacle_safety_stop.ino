
#include <Arduino.h>

// ============================================================
// TI-RSLK MAX - STAGE 3C
// CLOSED-LOOP CONTROL + THREE STOPS + OBSTACLE SLOWDOWN + SAFETY STOP
//
// Sequence:
//   Drive for 3 seconds
//   Stop for 1 second
//   Repeat until three stops are completed, then continue driving
//   straight indefinitely until the user sends x.
//
// Obstacle behaviour:
//   Distance <= 40 cm: reduce speed to 60 mm/s
//   Distance <= 15 cm: stop both motors
//   Safety stop clears at >= 20 cm
//   Slow mode clears at >= 50 cm
//   The 3-second driving timer pauses during a safety stop.
//
// Serial commands:
//   w = Start/restart sequence
//   x = Stop and cancel
//   + = Increase normal target speed
//   - = Decrease normal target speed
//   r = Reset encoder counts
//   p = Toggle telemetry
//   After the third 1-second stop, the robot continues straight.
//   x = Stop the robot at any time.
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

const bool LEFT_ENC_INVERTED  = true;
const bool RIGHT_ENC_INVERTED = false;

// -------------------- ULTRASONIC PINS -----------------------

const uint8_t TRIG_PIN = 9;
const uint8_t ECHO_PIN = 7;

// -------------------- WHEEL PARAMETERS ----------------------

const float WHEEL_DIAMETER_MM = 70.0f;
const float COUNTS_PER_REV = 720.0f;

const float MM_PER_TICK =
  (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

// -------------------- PI CONTROLLER -------------------------

float KP = 0.20f;
float KI = 0.50f;

float targetSpeed = 120.0f;

const float OBSTACLE_SPEED = 60.0f;

const float BASE_PWM = 65.0f;

const int PWM_MIN = 0;
const int PWM_MAX = 150;

const float INTEGRAL_LIMIT = 60.0f;

// -------------------- OBSTACLE SETTINGS ---------------------

const float OBSTACLE_DETECT_CM = 40.0f;
const float OBSTACLE_CLEAR_CM = 50.0f;

const float SAFETY_STOP_CM = 15.0f;
const float SAFETY_RESUME_CM = 20.0f;

const uint32_t SENSOR_INTERVAL_MS = 100;
const uint32_t ECHO_TIMEOUT_US = 25000;

float distanceCm = -1.0f;

bool obstacleDetected = false;
bool safetyStopActive = false;

// Used to pause the 3-second driving timer only when a safety
// stop interrupts the DRIVING state.
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

  int8_t d =
    (digitalRead(PIN_ENC_L_A) != digitalRead(PIN_ENC_L_B))
    ? 1 : -1;

  ticksL += LEFT_ENC_INVERTED ? -d : d;
}

void isrRight() {

  int8_t d =
    (digitalRead(PIN_ENC_R_A) != digitalRead(PIN_ENC_R_B))
    ? 1 : -1;

  ticksR += RIGHT_ENC_INVERTED ? -d : d;
}

long readTicks(volatile long &counter) {

  noInterrupts();
  long value = counter;
  interrupts();

  return value;
}

// -------------------- MOTOR OUTPUT VARIABLES ----------------

int pwmL = 0;
int pwmR = 0;

// -------------------- MOTOR CONTROL -------------------------

void driveMotor(
  uint8_t pwmPin,
  uint8_t dirPin,
  bool inverted,
  int pwm
) {

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

  driveMotor(
    PIN_PWM_L,
    PIN_DIR_L,
    LEFT_MOTOR_INVERTED,
    0
  );

  driveMotor(
    PIN_PWM_R,
    PIN_DIR_R,
    RIGHT_MOTOR_INVERTED,
    0
  );
}

// -------------------- PI CONTROLLER -------------------------

float integralL = 0.0f;
float integralR = 0.0f;

int calculatePI(
  float target,
  float measured,
  float &integral,
  float dt
) {

  float error = target - measured;

  float candidateIntegral =
    integral + KI * error * dt;

  candidateIntegral = constrain(
    candidateIntegral,
    -INTEGRAL_LIMIT,
    INTEGRAL_LIMIT
  );

  float output =
    BASE_PWM + KP * error + candidateIntegral;

  float limitedOutput = constrain(
    output,
    (float)PWM_MIN,
    (float)PWM_MAX
  );

  if (output == limitedOutput ||
      (output > PWM_MAX && error < 0) ||
      (output < PWM_MIN && error > 0)) {

    integral = candidateIntegral;
  }

  return (int)constrain(
    limitedOutput,
    (float)PWM_MIN,
    (float)PWM_MAX
  );
}

// -------------------- ULTRASONIC MEASUREMENT ----------------

float readDistanceCm() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    ECHO_TIMEOUT_US
  );

  if (duration == 0) {
    return -1.0f;
  }

  return (duration * 0.0343f) / 2.0f;
}

// -------------------- ROBOT STATE MACHINE -------------------

enum RobotState {
  IDLE,
  DRIVING,
  STOPPED,
  FINISHED
};

RobotState robotState = IDLE;

uint8_t completedStops = 0;

uint32_t stateStartMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;

long previousTicksL = 0;
long previousTicksR = 0;

float velocityL = 0.0f;
float velocityR = 0.0f;

bool telemetryOn = true;

// -------------------- OBSTACLE DETECTION --------------------

void updateObstacleDetection() {

  uint32_t now = millis();

  if (now - lastSensorReadMs < SENSOR_INTERVAL_MS) {
    return;
  }

  lastSensorReadMs = now;
  distanceCm = readDistanceCm();

  // A missing echo is NOT treated as a clear path.
  // If the safety stop is active, the robot remains stopped
  // until a valid reading reaches the resume threshold.

  if (distanceCm >= 0.0f) {

    // Enter safety-stop mode at 15 cm or closer.
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

      Serial.println();
      Serial.println(F("# SAFETY STOP - obstacle at 15 cm or closer"));
      Serial.println(F("# Motors stopped; waiting for at least 20 cm clearance"));
    }

    // Resume only after a valid reading reaches 20 cm.
    else if (safetyStopActive && distanceCm >= SAFETY_RESUME_CM) {

      safetyStopActive = false;

      if (safetyStopPausedDriveTimer && robotState == DRIVING) {
        // Exclude the safety-stop duration from the 3-second
        // active driving interval.
        stateStartMs += (now - safetyStopStartMs);
      }

      safetyStopPausedDriveTimer = false;

      integralL = 0.0f;
      integralR = 0.0f;
      velocityL = 0.0f;
      velocityR = 0.0f;

      previousTicksL = readTicks(ticksL);
      previousTicksR = readTicks(ticksR);
      lastControlMs = now;

      Serial.println();
      Serial.println(F("# SAFETY STOP CLEARED"));
      Serial.println(F("# Resuming motion if sequence is active"));
    }

    // Enter reduced-speed mode.
    if (!obstacleDetected &&
        distanceCm <= OBSTACLE_DETECT_CM) {

      obstacleDetected = true;

      Serial.println();
      Serial.println(F("# OBSTACLE DETECTED"));
      Serial.println(F("# Reducing target speed to 60 mm/s"));
    }

    // Exit reduced-speed mode only after a valid clear reading.
    else if (obstacleDetected &&
             distanceCm >= OBSTACLE_CLEAR_CM) {

      obstacleDetected = false;

      Serial.println();
      Serial.println(F("# PATH CLEAR"));
      Serial.println(F("# Restoring normal speed"));
    }
  }
}

// -------------------- ACTIVE TARGET SPEED ------------------

float getActiveTargetSpeed() {

  if (obstacleDetected) {
    return min(targetSpeed, OBSTACLE_SPEED);
  }

  return targetSpeed;
}



// -------------------- RESET DRIVE CONTROLLER ---------------

void resetDriveController() {

  integralL = 0.0f;
  integralR = 0.0f;

  pwmL = 0;
  pwmR = 0;

  previousTicksL = readTicks(ticksL);
  previousTicksR = readTicks(ticksR);

  velocityL = 0.0f;
  velocityR = 0.0f;

  lastControlMs = millis();
}

// -------------------- START SEQUENCE ------------------------

void startSequence() {

  stopMotors();

  completedStops = 0;

  resetDriveController();

  robotState = DRIVING;
  stateStartMs = millis();

  Serial.println();
  Serial.println(F("# Stage 3C sequence started"));
  Serial.println(F("# Normal speed: 120 mm/s"));
  Serial.println(F("# Obstacle speed: 60 mm/s"));
  Serial.println(F("# Safety stop: 15 cm; resume: 20 cm"));
}

// -------------------- CANCEL SEQUENCE -----------------------

void cancelSequence() {

  stopMotors();

  integralL = 0.0f;
  integralR = 0.0f;

  robotState = IDLE;
  completedStops = 0;

  velocityL = 0.0f;
  velocityR = 0.0f;

  Serial.println(F("# Sequence cancelled - robot stopped"));
}

// -------------------- STATE MACHINE UPDATE ------------------

void updateSequence(uint32_t now) {

  if (robotState == DRIVING &&
      !safetyStopActive &&
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

  else if (robotState == STOPPED &&
           now - stateStartMs >= STOP_DURATION_MS) {

    if (completedStops < TOTAL_STOPS) {

      resetDriveController();

      robotState = DRIVING;
      stateStartMs = now;

      Serial.println(F("# Resuming forward movement"));
    }

    else {

      // The three-stop requirement is complete. Do not stop the
      // robot permanently: reset the PI controller and continue
      // driving straight at the currently selected target speed.
      resetDriveController();
      robotState = FINISHED;

      Serial.println();
      Serial.println(F("# ==============================="));
      Serial.println(F("# THREE-STOP SEQUENCE COMPLETED"));
      Serial.println(F("# Continuing straight at target speed"));
      Serial.println(F("# Obstacle slowdown and safety stop remain active"));
      Serial.println(F("# Press x to stop the robot"));
      Serial.println(F("# ==============================="));
    }
  }
}

// -------------------- SERIAL COMMANDS -----------------------

void handleSerial() {

  while (Serial.available()) {

    char c = Serial.read();

    switch (c) {

      case 'w':
      case 'W':
        startSequence();
        break;

      case 'x':
      case 'X':
      case ' ':
        cancelSequence();
        break;

      case '+':
      case '=':
        targetSpeed = min(250.0f, targetSpeed + 10.0f);

        Serial.print(F("# Normal target speed: "));
        Serial.println(targetSpeed);
        break;

      case '-':
      case '_':
        targetSpeed = max(40.0f, targetSpeed - 10.0f);

        Serial.print(F("# Normal target speed: "));
        Serial.println(targetSpeed);
        break;

      case 'r':
      case 'R':

        noInterrupts();
        ticksL = 0;
        ticksR = 0;
        interrupts();

        previousTicksL = 0;
        previousTicksR = 0;

        Serial.println(F("# Encoder counts reset"));
        break;

      case 'p':
      case 'P':
        telemetryOn = !telemetryOn;

        Serial.print(F("# Telemetry "));
        Serial.println(telemetryOn ? F("ON") : F("OFF"));
        break;
    }
  }
}

// -------------------- SETUP ---------------------------------

void setup() {

  Serial.begin(115200);

  pinMode(PIN_PWM_L, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);

  pinMode(PIN_PWM_R, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);

  pinMode(PIN_SLEEP_L, OUTPUT);
  pinMode(PIN_SLEEP_R, OUTPUT);

  analogWrite(PIN_PWM_L, 0);
  analogWrite(PIN_PWM_R, 0);

  digitalWrite(PIN_SLEEP_L, HIGH);
  digitalWrite(PIN_SLEEP_R, HIGH);

  pinMode(PIN_ENC_L_A, INPUT);
  pinMode(PIN_ENC_L_B, INPUT);

  pinMode(PIN_ENC_R_A, INPUT);
  pinMode(PIN_ENC_R_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(PIN_ENC_L_A),
    isrLeft,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(PIN_ENC_R_A),
    isrRight,
    CHANGE
  );

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  lastControlMs = millis();
  lastTelemetryMs = millis();
  lastSensorReadMs = millis();

  Serial.println();
  Serial.println(F("# TI-RSLK MAX - Stage 3C"));
  Serial.println(F("# PI control + three stops + obstacle slowdown + safety stop"));
  Serial.println(F("# HC-SR04: TRIG D9 | ECHO D7"));
  Serial.println(F("# Slow down at: 40 cm"));
  Serial.println(F("# Safety stop at: 15 cm or closer"));
  Serial.println(F("# Safety resume at: 20 cm or farther"));
  Serial.println(F("# Return to normal speed at: 50 cm or farther"));
  Serial.println(F("# Commands: w=start, x=stop, +/-=speed"));
  Serial.println(F("# r=reset encoders, p=telemetry"));
  Serial.println(F("# Send w to begin"));
}

// -------------------- MAIN LOOP -----------------------------

void loop() {

  handleSerial();

  uint32_t now = millis();

  // Continuously monitor the ultrasonic sensor.
  updateObstacleDetection();

  // Update three-stop sequence.
  updateSequence(now);

  // PI control during the timed sequence AND after the three-stop
  // sequence has completed. FINISHED means continuous-drive mode.
  if ((robotState == DRIVING || robotState == FINISHED) &&
      !safetyStopActive) {

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

      pwmL = calculatePI(
        activeTargetSpeed,
        velocityL,
        integralL,
        dt
      );

      pwmR = calculatePI(
        activeTargetSpeed,
        velocityR,
        integralR,
        dt
      );

      driveMotor(
        PIN_PWM_L,
        PIN_DIR_L,
        LEFT_MOTOR_INVERTED,
        pwmL
      );

      driveMotor(
        PIN_PWM_R,
        PIN_DIR_R,
        RIGHT_MOTOR_INVERTED,
        pwmR
      );
    }
  }

  // ---------------- TELEMETRY ----------------

  now = millis();

  if (telemetryOn &&
      (robotState == DRIVING || robotState == STOPPED ||
       robotState == FINISHED) &&
      now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {

    lastTelemetryMs = now;

    Serial.print(F("TARGET,"));
    Serial.print(safetyStopActive ? 0.0f : getActiveTargetSpeed());

    Serial.print(F(",DIST_CM,"));

    if (distanceCm < 0.0f) {
      Serial.print(F("NO_ECHO"));
    }
    else {
      Serial.print(distanceCm, 2);
    }

    Serial.print(F(",MODE,"));
    if (safetyStopActive) {
      Serial.print(F("SAFETY_STOP"));
    }
    else if (robotState == FINISHED) {
      Serial.print(obstacleDetected ? F("CONTINUOUS_SLOW") : F("CONTINUOUS_NORMAL"));
    }
    else {
      Serial.print(obstacleDetected ? F("SLOW") : F("NORMAL"));
    }

    Serial.print(F(",VL,"));
    Serial.print(velocityL);

    Serial.print(F(",VR,"));
    Serial.print(velocityR);

    Serial.print(F(",PWM_L,"));
    Serial.print(pwmL);

    Serial.print(F(",PWM_R,"));
    Serial.print(pwmR);

    Serial.print(F(",TICKS_L,"));
    Serial.print(readTicks(ticksL));

    Serial.print(F(",TICKS_R,"));
    Serial.println(readTicks(ticksR));
  }
}
