
#include <Arduino.h>

// ============================================================
// TI-RSLK MAX - STAGE 3A: HC-SR04 DISTANCE SENSOR TEST
// Arduino Uno
//
// HC-SR04 connections:
//   VCC  -> 5V
//   GND  -> GND
//   TRIG -> D9
//   ECHO -> D7
//
// Purpose:
//   Measure distance and display it in Serial Monitor.
//   No motor control or obstacle avoidance in this stage.
// ============================================================

// -------------------- ULTRASONIC PINS -----------------------

const uint8_t TRIG_PIN = 9;
const uint8_t ECHO_PIN = 7;

// -------------------- SENSOR SETTINGS -----------------------

const uint32_t SENSOR_INTERVAL_MS = 100;
const uint32_t ECHO_TIMEOUT_US = 30000;

// -------------------- VARIABLES -----------------------------

uint32_t lastSensorReadMs = 0;

float distanceCm = 0.0f;

// -------------------- DISTANCE MEASUREMENT ------------------

float readDistanceCm() {

  // Ensure trigger starts LOW.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond trigger pulse.
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the ECHO pulse duration.
  unsigned long duration = pulseIn(
    ECHO_PIN,
    HIGH,
    ECHO_TIMEOUT_US
  );

  // No valid echo received.
  if (duration == 0) {
    return -1.0f;
  }

  // Speed of sound conversion:
  // Distance (cm) = duration * 0.0343 / 2
  float distance =
    (duration * 0.0343f) / 2.0f;

  return distance;
}

// -------------------- SETUP ---------------------------------

void setup() {

  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  Serial.println();
  Serial.println(F("# TI-RSLK MAX - Stage 3A"));
  Serial.println(F("# HC-SR04 Ultrasonic Sensor Test"));
  Serial.println(F("# TRIG: D9 | ECHO: D7"));
  Serial.println(F("# Distance readings in centimetres"));
  Serial.println();

  Serial.println(F("TIME_MS,DISTANCE_CM"));
}

// -------------------- MAIN LOOP -----------------------------

void loop() {

  uint32_t now = millis();

  if (now - lastSensorReadMs >= SENSOR_INTERVAL_MS) {

    lastSensorReadMs = now;

    distanceCm = readDistanceCm();

    Serial.print(now);
    Serial.print(',');

    if (distanceCm < 0.0f) {
      Serial.println(F("NO_ECHO"));
    }
    else {
      Serial.println(distanceCm, 2);
    }
  }
}
