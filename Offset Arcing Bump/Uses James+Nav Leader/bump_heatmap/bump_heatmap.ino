#include "BumpSensors.h"

BumpSensors_c bump_sensors;

// --------------------------------------------------------------------------
// SETTINGS
// --------------------------------------------------------------------------

const int BUTTON_A_PIN = 14;
const unsigned long SENSOR_TIMEOUT_US = 3000;

const int BASELINE_SAMPLES = 80;
const int BASELINE_SIGNAL_SAMPLES = 50;

// --------------------------------------------------------------------------
// SIGNAL VARIABLES
// --------------------------------------------------------------------------

float bump_baseline[BUMP_NUM_SENSORS];
float baselineTotal = 0.0f;

float L = 0.0f;
float R = 0.0f;
float total = 0.0f;

bool lastButtonState = HIGH;
int sampleIndex = 0;

// --------------------------------------------------------------------------
// HELPERS
// --------------------------------------------------------------------------

float readingToSignal(float reading, float baseline) {
  if (baseline <= 1.0f) return 0.0f;

  float x = 100.0f * (baseline - reading) / baseline;

  if (x < 0.0f) x = 0.0f;
  if (x > 100.0f) x = 100.0f;
  return x;
}

void measureBumpBaseline() {
  for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
    bump_baseline[s] = 0.0f;
  }

  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    bump_sensors.readSensorsDigital();
    for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
      bump_baseline[s] += bump_sensors.readings[s];
    }
    delay(5);
  }

  for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
    bump_baseline[s] /= (float)BASELINE_SAMPLES;
  }

  baselineTotal = 0.0f;
  for (int i = 0; i < BASELINE_SIGNAL_SAMPLES; i++) {
    bump_sensors.readSensorsDigital();

    float Lb = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
    float Rb = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

    baselineTotal += (Lb + Rb);
    delay(5);
  }

  baselineTotal /= (float)BASELINE_SIGNAL_SAMPLES;
}

void updateSignals() {
  bump_sensors.readSensorsDigital();

  L = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
  R = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);
  total = L + R;
}

void settleSignals(int nSamples, int delayMs) {
  for (int i = 0; i < nSamples; i++) {
    updateSignals();
    delay(delayMs);
  }
}

void printOneSample() {
  Serial.print(sampleIndex);
  Serial.print(",");
  Serial.print(total, 3);
  Serial.print(",");
  Serial.print(L, 3);
  Serial.print(",");
  Serial.println(R, 3);

  sampleIndex++;
}

// --------------------------------------------------------------------------
// SETUP / LOOP
// --------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  bump_sensors.initialiseForDigital();
  bump_sensors.timeout_us = SENSOR_TIMEOUT_US;

  pinMode(EMIT_PIN, INPUT);

  delay(500);
  measureBumpBaseline();

  Serial.println("sample_index,total,L,R");
}

void loop() {
  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (lastButtonState == HIGH && buttonState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW) {
      // Let the reading settle at the current leader position
      settleSignals(10, 20);   // about 200 ms settle
      printOneSample();
    }
  }

  lastButtonState = buttonState;
}
