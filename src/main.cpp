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
SensorState CurrentSensorState = WAIT_FOR_S1;
bool Sensor1PreviouslyActive = false;
bool Sensor2PreviouslyActive = false;
unsigned long Sensor1TimestampUs = 0;
unsigned long Sensor2TimestampUs = 0;

// Belt position in steps
long CurrentSteps = 0;

// Returns how many motor steps move the cart by 1 mm.
float stepsPerMm() {
  return (static_cast<float>(STEPS_PER_REV) * static_cast<float>(MICROSTEPS)) / BELT_MM_PER_REV;
}

// Reads one IR sensor and returns true when it sees an object.
bool sensorActive(uint8_t PinNumber) {
  int SensorPinState = digitalRead(PinNumber);
  return SENSOR_ACTIVE_LOW ? (SensorPinState == LOW) : (SensorPinState == HIGH);
}

// Reads the limit switch and returns true when it is pressed.
bool limitActive() {
  int LimitPinState = digitalRead(LIMIT_PIN);
  return LIMIT_ACTIVE_LOW ? (LimitPinState == LOW) : (LimitPinState == HIGH);
}

// Sets motor direction toward or away from the table edge.
void setDir(bool AwayFromEdge) {
  digitalWrite(DIR_PIN, (AwayFromEdge == DIR_AWAY_HIGH) ? HIGH : LOW);
}

// Sends one step pulse so the motor moves a tiny amount.
void stepPulse(unsigned long HalfPulseUs) {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(HalfPulseUs);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(HalfPulseUs);
}

// Keeps speed inside the allowed min and max range.
unsigned int clampFreq(float FrequencyHz) {
  if (FrequencyHz < static_cast<float>(MIN_FREQ)) {
    return MIN_FREQ;
  }
  if (FrequencyHz > static_cast<float>(MAX_FREQ)) {
    return MAX_FREQ;
  }
  return static_cast<unsigned int>(FrequencyHz);
}

// Moves the motor by a step amount with a smooth start and stop.
void moveSteps(long StepDelta, unsigned int FrequencyHz) {
  if (StepDelta == 0) {
    return;
  }

  // Pick safe speed values before moving.
  FrequencyHz = clampFreq(static_cast<float>(FrequencyHz));
  unsigned int StartFrequencyHz = clampFreq(static_cast<float>(START_FREQ));
  if (StartFrequencyHz > FrequencyHz) {
    StartFrequencyHz = FrequencyHz;
  }
  unsigned long AccelSteps = ACCEL_STEPS;

  // Set direction and total move distance.
  bool MoveAwayFromEdge = (StepDelta > 0);
  setDir(MoveAwayFromEdge);

  long TotalSteps = (StepDelta > 0) ? StepDelta : -StepDelta;
  // If the move is short, reduce ramp distance to fit.
  if (TotalSteps < 2 * static_cast<long>(AccelSteps)) {
    AccelSteps = TotalSteps / 2;
  }
  if (TotalSteps <= static_cast<long>(SHORT_MOVE_STEPS) && AccelSteps > SHORT_MOVE_ACCEL_STEPS) {
    AccelSteps = SHORT_MOVE_ACCEL_STEPS;
    if (TotalSteps < 2 * static_cast<long>(AccelSteps)) {
      AccelSteps = TotalSteps / 2;
    }
  }

  if (AccelSteps == 0) {
    // No ramp needed: run at one constant speed.
    unsigned long StepPeriodUs = 1000000UL / FrequencyHz;
    for (long StepIndex = 0; StepIndex < TotalSteps; StepIndex++) {
      stepPulse(StepPeriodUs / 2);
    }
    CurrentSteps += StepDelta;
    return;
  }

  for (long StepIndex = 0; StepIndex < TotalSteps; StepIndex++) {
    // Ramp up at the start and ramp down near the end.
    unsigned int StepFrequencyHz = FrequencyHz;
    if (StepIndex < static_cast<long>(AccelSteps)) {
      float RampRatio = static_cast<float>(StepIndex + 1) / static_cast<float>(AccelSteps);
      StepFrequencyHz = clampFreq(StartFrequencyHz + (FrequencyHz - StartFrequencyHz) * RampRatio);
    } else if (StepIndex >= TotalSteps - static_cast<long>(AccelSteps)) {
      long RemainingSteps = TotalSteps - StepIndex;
      float RampRatio = static_cast<float>(RemainingSteps) / static_cast<float>(AccelSteps);
      StepFrequencyHz = clampFreq(StartFrequencyHz + (FrequencyHz - StartFrequencyHz) * RampRatio);
    }

    unsigned long StepPeriodUs = 1000000UL / StepFrequencyHz;
    stepPulse(StepPeriodUs / 2);
  }

  CurrentSteps += StepDelta;
}

// Picks a speed that should finish before the available time ends.
unsigned int freqFromTime(long StepsNeeded, float TimeAvailableS) {
  // Handle edge cases first.
  if (StepsNeeded <= 0) {
    return MIN_FREQ;
  }
  if (TimeAvailableS <= 0.0001f) {
    return MAX_FREQ;
  }
  // Compute needed speed and keep it in safe bounds.
  float NeededFrequencyHz = static_cast<float>(StepsNeeded) / TimeAvailableS;
  if (NeededFrequencyHz < static_cast<float>(CRUISE_FREQ)) {
    NeededFrequencyHz = static_cast<float>(CRUISE_FREQ);
  }
  return clampFreq(NeededFrequencyHz);
}

// Moves to the home switch so position can reset to zero.
void homeToSwitch() {
  long MaxSteps = lroundf((MAX_TRAVEL_MM + HOMING_EXTRA_MM) * stepsPerMm());

  // If we already start on the switch, move off it first.
  if (limitActive()) {
    moveSteps(HOMING_CLEAR_STEPS, HOMING_FREQ);
  }

  // Move toward the switch until it stays active for several reads.
  setDir(false);
  long HomingSteps = 0;
  int StableCount = 0;
  unsigned int StartFrequencyHz = clampFreq(static_cast<float>(HOMING_START_FREQ));
  unsigned int TargetFrequencyHz = clampFreq(static_cast<float>(HOMING_FREQ));
  while (HomingSteps < MaxSteps) {
    if (limitActive()) {
      StableCount++;
    } else {
      StableCount = 0;
    }
    if (StableCount >= LIMIT_STABLE_COUNT) {
      break;
    }

    float RampRatio = 1.0f;
    if (HOMING_ACCEL_STEPS > 0 && HomingSteps < static_cast<long>(HOMING_ACCEL_STEPS)) {
      RampRatio = static_cast<float>(HomingSteps + 1) / static_cast<float>(HOMING_ACCEL_STEPS);
    }
    unsigned int StepFrequencyHz = clampFreq(StartFrequencyHz + (TargetFrequencyHz - StartFrequencyHz) * RampRatio);
    unsigned long StepPeriodUs = 1000000UL / StepFrequencyHz;
    stepPulse(StepPeriodUs / 2);
    HomingSteps++;
  }

  // Stop if homing did not find the switch in expected travel.
  if (StableCount < LIMIT_STABLE_COUNT) {
    if (DEBUG_SERIAL) {
      Serial.println("Homing failed: switch not reached.");
    }
    return;
  }

  // Home is now our zero position.
  CurrentSteps = 0;
}

// Uses sensor timing to predict landing spot, move there, then go home.
void handleMeasurement(unsigned long Sensor1TimeUs, unsigned long Sensor2TimeUs) {
  // Reject bad timing input.
  if (Sensor2TimeUs <= Sensor1TimeUs) {
    return;
  }

  unsigned long DeltaUs = Sensor2TimeUs - Sensor1TimeUs;
  if (DeltaUs < MIN_VALID_DELTA_US) {
    return;
  }

  // Convert sensor timing into object speed.
  float DeltaS = static_cast<float>(DeltaUs) / 1000000.0f;
  float SpeedMps = (SENSOR_SPACING_MM / 1000.0f) / DeltaS;

  // Estimate where the object will land.
  float HeightM = TABLE_HEIGHT_MM / 1000.0f;
  float FallTimeS = sqrtf((2.0f * HeightM) / GRAVITY_MPS2);
  float LandingMm = SpeedMps * FallTimeS * 1000.0f;

  // Convert landing point into cart target and keep it in travel range.
  float CupRadiusMm = CUP_DIAMETER_MM * 0.5f;
  float TargetMm = (LandingMm - CupRadiusMm) + CUP_EDGE_PAST_TABLE_MM + TARGET_OFFSET_MM;
  if (TargetMm < 0.0f) {
    TargetMm = 0.0f;
  } else if (TargetMm > MAX_TRAVEL_MM) {
    TargetMm = MAX_TRAVEL_MM;
  }

  long TargetSteps = lroundf(TargetMm * stepsPerMm());
  long StepDelta = TargetSteps - CurrentSteps;
  long StepsNeeded = (StepDelta >= 0) ? StepDelta : -StepDelta;

  // Skip move if already at target.
  if (StepsNeeded == 0) {
    return;
  }

  // Choose speed based on how much time is left before landing.
  float TimeToEdgeS = 0.0f;
  if (SpeedMps > 0.01f) {
    TimeToEdgeS = (SENSOR2_TO_EDGE_MM / 1000.0f) / SpeedMps;
  }
  float TimeAvailableS = TimeToEdgeS + FallTimeS;
  unsigned int MoveFrequencyHz = freqFromTime(StepsNeeded, TimeAvailableS);

  if (DEBUG_SERIAL) {
    Serial.print("Speed m/s: ");
    Serial.print(SpeedMps, 3);
    Serial.print(" Landing mm: ");
    Serial.print(LandingMm, 1);
    Serial.print(" Target mm: ");
    Serial.print(TargetMm, 1);
    Serial.print(" Steps: ");
    Serial.print(StepsNeeded);
    Serial.print(" Freq: ");
    Serial.println(MoveFrequencyHz);
  }

  // Make the catch move, pause briefly, then return home.
  moveSteps(StepDelta, MoveFrequencyHz);

  if (CATCH_PAUSE_MS > 0) {
    delay(CATCH_PAUSE_MS);
  }
  homeToSwitch();
}

// Tracks sensor order and starts a catch after both triggers are seen.
void updateSensors() {
  bool Sensor1ActiveNow = sensorActive(SENSOR1_PIN);
  bool Sensor2ActiveNow = sensorActive(SENSOR2_PIN);

  // Move through the trigger order: sensor 1, sensor 2, then clear.
  switch (CurrentSensorState) {
    case WAIT_FOR_S1:
      if (Sensor1ActiveNow && !Sensor1PreviouslyActive) {
        Sensor1TimestampUs = micros();
        CurrentSensorState = WAIT_FOR_S2;
      }
      break;
    case WAIT_FOR_S2:
      if (Sensor2ActiveNow && !Sensor2PreviouslyActive) {
        Sensor2TimestampUs = micros();
        handleMeasurement(Sensor1TimestampUs, Sensor2TimestampUs);
        CurrentSensorState = WAIT_FOR_CLEAR;
      }
      break;
    case WAIT_FOR_CLEAR:
      if (!Sensor1ActiveNow && !Sensor2ActiveNow) {
        CurrentSensorState = WAIT_FOR_S1;
      }
      break;
  }

  // Save current sensor states for edge detection.
  Sensor1PreviouslyActive = Sensor1ActiveNow;
  Sensor2PreviouslyActive = Sensor2ActiveNow;
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
  static unsigned long LastLimitLog = 0;
  if (DEBUG_SERIAL) {
    // Print switch state at a fixed interval.
    unsigned long CurrentTimeMs = millis();
    if (CurrentTimeMs - LastLimitLog >= LIMIT_LOG_INTERVAL_MS) {
      LastLimitLog = CurrentTimeMs;
      Serial.print("Limit switch: ");
      Serial.println(limitActive() ? "ACTIVE" : "INACTIVE");
    }
  }
  // Keep watching sensors for the next catch cycle.
  updateSensors();
}
//Code by Orion North
//Templeton STEM Senior Project #1 (2026) - CatchBOT

