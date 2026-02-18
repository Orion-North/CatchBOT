//Code by Orion North
//Templeton STEM Senior Project #1 (2026) - CatchBOT
#include <Arduino.h>
#include <math.h>

// CatchBOT controller
// Pins
#define DIR_PIN 2
#define STEP_PIN 3
#define ENABLE_PIN -1

// Motion limits
const unsigned int MAX_FREQ = 4000;
const unsigned int LOW_SPEED_FLOOR = 700;
const unsigned int MIN_FREQ = LOW_SPEED_FLOOR;
const unsigned int CRUISE_FREQ = 1000;

// IR sensors
const uint8_t SENSOR1_PIN = 10;
const uint8_t SENSOR2_PIN = 9;

// Limit switch
const uint8_t LIMIT_PIN = 8;

// Input polarity
const bool SENSOR_ACTIVE_LOW = true;
const bool SENSOR_USE_PULLUP = true;
const bool LIMIT_ACTIVE_LOW = false;
const bool LIMIT_USE_PULLUP = true;

// Geometry calibration
const float SENSOR_SPACING_MM = 38.0f;
float TABLE_HEIGHT_MM = 720.0f;
float SENSOR2_TO_EDGE_MM = 6.0f;
const float CUP_DIAMETER_MM = 50.0f;
const float CUP_EDGE_PAST_TABLE_MM = 5.0f;
const float TARGET_OFFSET_MM = -50.0f;

// Belt and steps
const int STEPS_PER_REV = 200;
const int MICROSTEPS = 1;
const float BELT_MM_PER_REV = 40.0f;
const float MAX_TRAVEL_MM = 340.0f;

// Direction setting
const bool DIR_AWAY_HIGH = false;

// Physics
const float GRAVITY_MPS2 = 9.80665f;

// Timing filter
const unsigned long MIN_VALID_DELTA_US = 200;

// Homing settings
const unsigned int HOMING_FREQ = CRUISE_FREQ;
const unsigned int HOMING_START_FREQ = CRUISE_FREQ;
const unsigned long HOMING_ACCEL_STEPS = 0;
const float HOMING_EXTRA_MM = 20.0f;
const int HOMING_CLEAR_STEPS = 50;
const int LIMIT_STABLE_COUNT = 5;

// Catch timing
const unsigned long CATCH_PAUSE_MS = 500;

// Motion profile
const unsigned int START_FREQ = LOW_SPEED_FLOOR;
const unsigned long ACCEL_STEPS = 160;
const unsigned long SHORT_MOVE_STEPS = 120;
const unsigned long SHORT_MOVE_ACCEL_STEPS = 24;

// Diagnostics
const bool DIAGNOSTIC_JOG = false;
const long DIAG_JOG_STEPS = 400;

// Debug
const bool DEBUG_SERIAL = true;
const unsigned long LIMIT_LOG_INTERVAL_MS = 250;

// Sensor state
enum SensorState {
  WAIT_FOR_S1,
  WAIT_FOR_S2,
  WAIT_FOR_CLEAR
};

// Runtime state
SensorState sensor_state = WAIT_FOR_S1;
bool s1_prev = false;
bool s2_prev = false;
unsigned long t1_us = 0;
unsigned long t2_us = 0;

// Belt position in steps
long current_steps = 0;

// Returns how many motor steps move the cart by 1 mm.
float stepsPerMm() {
  return (static_cast<float>(STEPS_PER_REV) * static_cast<float>(MICROSTEPS)) / BELT_MM_PER_REV;
}

// Reads one IR sensor and returns true when it sees an object.
bool sensorActive(uint8_t pin) {
  int v = digitalRead(pin);
  return SENSOR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Reads the limit switch and returns true when it is pressed.
bool limitActive() {
  int v = digitalRead(LIMIT_PIN);
  return LIMIT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Sets motor direction toward or away from the table edge.
void setDir(bool away_from_edge) {
  digitalWrite(DIR_PIN, (away_from_edge == DIR_AWAY_HIGH) ? HIGH : LOW);
}

// Sends one step pulse so the motor moves a tiny amount.
void stepPulse(unsigned long half_pulse) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(half_pulse);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(half_pulse);
}

// Keeps speed inside the allowed min and max range.
unsigned int clampFreq(float freq) {
  if (freq < static_cast<float>(MIN_FREQ)) {
    return MIN_FREQ;
  }
  if (freq > static_cast<float>(MAX_FREQ)) {
    return MAX_FREQ;
  }
  return static_cast<unsigned int>(freq);
}

// Moves the motor by a step amount with a smooth start and stop.
void moveSteps(long step_delta, unsigned int freq) {
  if (step_delta == 0) {
    return;
  }

  // Pick safe speed values before moving.
  freq = clampFreq(static_cast<float>(freq));
  unsigned int start_freq = clampFreq(static_cast<float>(START_FREQ));
  if (start_freq > freq) {
    start_freq = freq;
  }
  unsigned long accel_steps = ACCEL_STEPS;

  // Set direction and total move distance.
  bool away = (step_delta > 0);
  setDir(away);

  long total_steps = (step_delta > 0) ? step_delta : -step_delta;
  // If the move is short, reduce ramp distance to fit.
  if (total_steps < 2 * static_cast<long>(accel_steps)) {
    accel_steps = total_steps / 2;
  }
  if (total_steps <= static_cast<long>(SHORT_MOVE_STEPS) && accel_steps > SHORT_MOVE_ACCEL_STEPS) {
    accel_steps = SHORT_MOVE_ACCEL_STEPS;
    if (total_steps < 2 * static_cast<long>(accel_steps)) {
      accel_steps = total_steps / 2;
    }
  }

  if (accel_steps == 0) {
    // No ramp needed: run at one constant speed.
    unsigned long period_us = 1000000UL / freq;
    for (long i = 0; i < total_steps; i++) {
      stepPulse(period_us / 2);
    }
    current_steps += step_delta;
    return;
  }

  for (long i = 0; i < total_steps; i++) {
    // Ramp up at the start and ramp down near the end.
    unsigned int f = freq;
    if (i < static_cast<long>(accel_steps)) {
      float t = static_cast<float>(i + 1) / static_cast<float>(accel_steps);
      f = clampFreq(start_freq + (freq - start_freq) * t);
    } else if (i >= total_steps - static_cast<long>(accel_steps)) {
      long d = total_steps - i;
      float t = static_cast<float>(d) / static_cast<float>(accel_steps);
      f = clampFreq(start_freq + (freq - start_freq) * t);
    }

    unsigned long period_us = 1000000UL / f;
    stepPulse(period_us / 2);
  }

  current_steps += step_delta;
}

// Picks a speed that should finish before the available time ends.
unsigned int freqFromTime(long steps_needed, float time_available_s) {
  // Handle edge cases first.
  if (steps_needed <= 0) {
    return MIN_FREQ;
  }
  if (time_available_s <= 0.0001f) {
    return MAX_FREQ;
  }
  // Compute needed speed and keep it in safe bounds.
  float needed_freq = static_cast<float>(steps_needed) / time_available_s;
  if (needed_freq < static_cast<float>(CRUISE_FREQ)) {
    needed_freq = static_cast<float>(CRUISE_FREQ);
  }
  return clampFreq(needed_freq);
}

// Moves to the home switch so position can reset to zero.
void homeToSwitch() {
  long max_steps = lroundf((MAX_TRAVEL_MM + HOMING_EXTRA_MM) * stepsPerMm());

  // If we already start on the switch, move off it first.
  if (limitActive()) {
    moveSteps(HOMING_CLEAR_STEPS, HOMING_FREQ);
  }

  // Move toward the switch until it stays active for several reads.
  setDir(false);
  long steps = 0;
  int stable = 0;
  unsigned int start_freq = clampFreq(static_cast<float>(HOMING_START_FREQ));
  unsigned int target_freq = clampFreq(static_cast<float>(HOMING_FREQ));
  while (steps < max_steps) {
    if (limitActive()) {
      stable++;
    } else {
      stable = 0;
    }
    if (stable >= LIMIT_STABLE_COUNT) {
      break;
    }

    float t = 1.0f;
    if (HOMING_ACCEL_STEPS > 0 && steps < static_cast<long>(HOMING_ACCEL_STEPS)) {
      t = static_cast<float>(steps + 1) / static_cast<float>(HOMING_ACCEL_STEPS);
    }
    unsigned int f = clampFreq(start_freq + (target_freq - start_freq) * t);
    unsigned long period_us = 1000000UL / f;
    stepPulse(period_us / 2);
    steps++;
  }

  // Stop if homing did not find the switch in expected travel.
  if (stable < LIMIT_STABLE_COUNT) {
    if (DEBUG_SERIAL) {
      Serial.println("Homing failed: switch not reached.");
    }
    return;
  }

  // Home is now our zero position.
  current_steps = 0;
}

// Uses sensor timing to predict landing spot, move there, then go home.
void handleMeasurement(unsigned long t1, unsigned long t2) {
  // Reject bad timing input.
  if (t2 <= t1) {
    return;
  }

  unsigned long delta_us = t2 - t1;
  if (delta_us < MIN_VALID_DELTA_US) {
    return;
  }

  // Convert sensor timing into object speed.
  float delta_s = static_cast<float>(delta_us) / 1000000.0f;
  float speed_mps = (SENSOR_SPACING_MM / 1000.0f) / delta_s;

  // Estimate where the object will land.
  float height_m = TABLE_HEIGHT_MM / 1000.0f;
  float fall_time_s = sqrtf((2.0f * height_m) / GRAVITY_MPS2);
  float landing_mm = speed_mps * fall_time_s * 1000.0f;

  // Convert landing point into cart target and keep it in travel range.
  float cup_radius_mm = CUP_DIAMETER_MM * 0.5f;
  float target_mm = (landing_mm - cup_radius_mm) + CUP_EDGE_PAST_TABLE_MM + TARGET_OFFSET_MM;
  if (target_mm < 0.0f) {
    target_mm = 0.0f;
  } else if (target_mm > MAX_TRAVEL_MM) {
    target_mm = MAX_TRAVEL_MM;
  }

  long target_steps = lroundf(target_mm * stepsPerMm());
  long step_delta = target_steps - current_steps;
  long steps_needed = (step_delta >= 0) ? step_delta : -step_delta;

  // Skip move if already at target.
  if (steps_needed == 0) {
    return;
  }

  // Choose speed based on how much time is left before landing.
  float time_to_edge_s = 0.0f;
  if (speed_mps > 0.01f) {
    time_to_edge_s = (SENSOR2_TO_EDGE_MM / 1000.0f) / speed_mps;
  }
  float time_available_s = time_to_edge_s + fall_time_s;
  unsigned int freq = freqFromTime(steps_needed, time_available_s);

  if (DEBUG_SERIAL) {
    Serial.print("Speed m/s: ");
    Serial.print(speed_mps, 3);
    Serial.print(" Landing mm: ");
    Serial.print(landing_mm, 1);
    Serial.print(" Target mm: ");
    Serial.print(target_mm, 1);
    Serial.print(" Steps: ");
    Serial.print(steps_needed);
    Serial.print(" Freq: ");
    Serial.println(freq);
  }

  // Make the catch move, pause briefly, then return home.
  moveSteps(step_delta, freq);

  if (CATCH_PAUSE_MS > 0) {
    delay(CATCH_PAUSE_MS);
  }
  homeToSwitch();
}

// Tracks sensor order and starts a catch after both triggers are seen.
void updateSensors() {
  bool s1 = sensorActive(SENSOR1_PIN);
  bool s2 = sensorActive(SENSOR2_PIN);

  // Move through the trigger order: sensor 1, sensor 2, then clear.
  switch (sensor_state) {
    case WAIT_FOR_S1:
      if (s1 && !s1_prev) {
        t1_us = micros();
        sensor_state = WAIT_FOR_S2;
      }
      break;
    case WAIT_FOR_S2:
      if (s2 && !s2_prev) {
        t2_us = micros();
        handleMeasurement(t1_us, t2_us);
        sensor_state = WAIT_FOR_CLEAR;
      }
      break;
    case WAIT_FOR_CLEAR:
      if (!s1 && !s2) {
        sensor_state = WAIT_FOR_S1;
      }
      break;
  }

  // Save current sensor states for edge detection.
  s1_prev = s1;
  s2_prev = s2;
}

// Runs once at startup to set pins, home the motor, and print status.
void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(115200);
  }

  // Set motor control pins.
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  // Enable output pin only when one is configured.
  if (ENABLE_PIN != -1) {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
  }

  setDir(true);

  // Set sensor pin modes.
  if (SENSOR_USE_PULLUP) {
    pinMode(SENSOR1_PIN, INPUT_PULLUP);
    pinMode(SENSOR2_PIN, INPUT_PULLUP);
  } else {
    pinMode(SENSOR1_PIN, INPUT);
    pinMode(SENSOR2_PIN, INPUT);
  }

  if (LIMIT_USE_PULLUP) {
    pinMode(LIMIT_PIN, INPUT_PULLUP);
  } else {
    pinMode(LIMIT_PIN, INPUT);
  }

  if (DEBUG_SERIAL) {
    Serial.print("Limit switch initial: ");
    Serial.println(limitActive() ? "ACTIVE" : "INACTIVE");
  }

  // Start from known home position.
  homeToSwitch();

  // Optional short back-and-forth test move.
  if (DIAGNOSTIC_JOG) {
    moveSteps(DIAG_JOG_STEPS, MIN_FREQ);
    delay(200);
    moveSteps(-DIAG_JOG_STEPS, MIN_FREQ);
  }

  if (DEBUG_SERIAL) {
    Serial.println("CatchBOT ready.");
  }
}

// Runs forever: logs switch state and keeps checking sensors.
void loop() {
  static unsigned long last_limit_log = 0;
  if (DEBUG_SERIAL) {
    // Print switch state at a fixed interval.
    unsigned long now = millis();
    if (now - last_limit_log >= LIMIT_LOG_INTERVAL_MS) {
      last_limit_log = now;
      Serial.print("Limit switch: ");
      Serial.println(limitActive() ? "ACTIVE" : "INACTIVE");
    }
  }
  // Keep watching sensors for the next catch cycle.
  updateSensors();
}
//Code by Orion North
//Templeton STEM Senior Project #1 (2026) - CatchBOT
