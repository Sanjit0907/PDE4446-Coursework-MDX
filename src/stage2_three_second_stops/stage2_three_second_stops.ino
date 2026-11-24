
#include <Arduino.h>

// ============================================================
// TI-RSLK MAX - STAGE 2: CLOSED-LOOP CONTROL WITH THREE STOPS
// Arduino Uno
//
// Sequence:
//   Drive 3 seconds
//   Stop  1 second
//   Drive 3 seconds
//   Stop  1 second
//   Drive 3 seconds
//   Stop  1 second
//   Finish
//
// Serial commands:
//   w = Start/restart complete sequence
//   x = Stop and cancel sequence
//   + = Increase target speed
//   - = Decrease target speed
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

// Preserve tested motor and encoder directions.

const bool LEFT_MOTOR_INVERTED  = false;
const bool RIGHT_MOTOR_INVERTED = false;

const bool LEFT_ENC_INVERTED  = true;
const bool RIGHT_ENC_INVERTED = false;

// -------------------- WHEEL PARAMETERS ----------------------

const float WHEEL_DIAMETER_MM = 70.0f;
const float COUNTS_PER_REV     = 720.0f;

const float MM_PER_TICK =
  (PI * WHEEL_DIAMETER_MM) / COUNTS_PER_REV;

// -------------------- PI CONTROLLER -------------------------

float KP = 0.20f;
float KI = 0.50f;

float targetSpeed = 120.0f;  // mm/s

const float BASE_PWM = 65.0f;

const int PWM_MIN = 0;
const int PWM_MAX = 150;

const float INTEGRAL_LIMIT = 60.0f;

// -------------------- SEQUENCE TIMING -----------------------

const uint32_t DRIVE_DURATION_MS = 3000;
const uint32_t STOP_DURATION_MS  = 1000;

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

  // Anti-windup
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
  Serial.println(F("# Sequence started"));
  Serial.println(F("# Driving for 3 seconds"));
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

  // Driving interval completed: enter stop state.

  if (robotState == DRIVING &&
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

  // Stop interval completed: resume or finish.

  else if (robotState == STOPPED &&
           now - stateStartMs >= STOP_DURATION_MS) {

    if (completedStops < TOTAL_STOPS) {

      resetDriveController();

      robotState = DRIVING;
      stateStartMs = now;

      Serial.println(F("# Resuming forward movement"));
    }
    else {

      stopMotors();

      robotState = FINISHED;

      Serial.println();
      Serial.println(F("# ================================="));
      Serial.println(F("# SEQUENCE COMPLETED"));
      Serial.println(F("# Three stops completed"));
      Serial.println(F("# Robot remains stopped"));
      Serial.println(F("# ================================="));
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

        Serial.print(F("# Target speed: "));
        Serial.println(targetSpeed);
        break;

      case '-':
      case '_':
        targetSpeed = max(40.0f, targetSpeed - 10.0f);

        Serial.print(F("# Target speed: "));
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

  lastControlMs = millis();
  lastTelemetryMs = millis();

  Serial.println();
  Serial.println(F("# TI-RSLK MAX - Stage 2"));
  Serial.println(F("# Closed-loop control with three stops"));
  Serial.println(F("# Drive: 3 seconds | Stop: 1 second"));

  Serial.println(F("# Commands:"));
  Serial.println(F("# w = Start/restart sequence"));
  Serial.println(F("# x = Cancel and stop"));
  Serial.println(F("# +/- = Adjust target speed"));
  Serial.println(F("# r = Reset encoder counts"));
  Serial.println(F("# p = Toggle telemetry"));

  Serial.println(F("# Send w to begin"));
}

// -------------------- MAIN LOOP -----------------------------

void loop() {

  handleSerial();

  uint32_t now = millis();

  // Update sequence state machine.
  updateSequence(now);

  // PI controller operates only during driving.

  if (robotState == DRIVING) {

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

      // Independent PI controllers.

      pwmL = calculatePI(
        targetSpeed,
        velocityL,
        integralL,
        dt
      );

      pwmR = calculatePI(
        targetSpeed,
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

  if (telemetryOn &&
      (robotState == DRIVING || robotState == STOPPED) &&
      now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {

    lastTelemetryMs = now;

    Serial.print(F("TARGET,"));
    Serial.print(targetSpeed);

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
