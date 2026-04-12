#include "LineSensors.h"

LineSensors_c line_sensors;

// --------------------------------------------------------------------------
// SETTINGS
// --------------------------------------------------------------------------

const int BUTTON_A_PIN = 14;

const int BASELINE_SAMPLES = 80;
const int BASELINE_SIGNAL_SAMPLES = 50;
const unsigned long SENSOR_TIMEOUT_US = 8000;

// --------------------------------------------------------------------------
// SIGNAL VARIABLES
// --------------------------------------------------------------------------

float line_baseline[NUM_SENSORS];
float baselineTotal = 0.0f;

float L = 0.0f;
float C = 0.0f;
float R = 0.0f;
float total = 0.0f;
float filteredTotal = 0.0f;

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

void measureLineBaseline() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    line_baseline[i] = 0.0f;
  }

  for (int n = 0; n < BASELINE_SAMPLES; n++) {
    line_sensors.readSensorsDigital();
    for (int i = 0; i < NUM_SENSORS; i++) {
      line_baseline[i] += line_sensors.readings[i];
    }
    delay(5);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    line_baseline[i] /= (float)BASELINE_SAMPLES;
  }

  baselineTotal = 0.0f;
  for (int n = 0; n < BASELINE_SIGNAL_SAMPLES; n++) {
    line_sensors.readSensorsDigital();

    float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
    float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
    float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
    float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);

    baselineTotal += (s0 + s1 + s3 + s4);
    delay(5);
  }

  baselineTotal /= (float)BASELINE_SIGNAL_SAMPLES;
  filteredTotal = baselineTotal;
}

void updateSignals() {
  line_sensors.readSensorsDigital();

  float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
  float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
  float s2 = readingToSignal(line_sensors.readings[2], line_baseline[2]);
  float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
  float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);

  L = s0 + s1;
  C = s2;
  R = s3 + s4;

  // Same definition as your follower code
  total = L + R;

  // Same filtering style as follower code
  filteredTotal = 0.7f * filteredTotal + 0.3f * total;
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
  Serial.print(filteredTotal, 3);
  Serial.print(",");
  Serial.print(L, 3);
  Serial.print(",");
  Serial.print(C, 3);
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

  line_sensors.initialiseForDigital();
  line_sensors.timeout_us = SENSOR_TIMEOUT_US;

  pinMode(EMIT_PIN, INPUT);

  delay(500);
  measureLineBaseline();

  Serial.println("sample_index,total,filteredTotal,L,C,R");
}

void loop() {
  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (lastButtonState == HIGH && buttonState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW) {
      // Let the filtered signal settle at the current leader position
      settleSignals(12, 20);   // about 240 ms of settling
      printOneSample();
    }
  }

  lastButtonState = buttonState;
}
