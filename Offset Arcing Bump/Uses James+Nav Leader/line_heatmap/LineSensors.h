#ifndef _LINESENSORS_H
#define _LINESENSORS_H

#define NUM_SENSORS 5

const int sensor_pins[NUM_SENSORS] = { A11, A0, A2, A3, A4 };

#define EMIT_PIN 11

enum ReadState { IDLE, CHARGING, DISCHARGING, DONE };

class LineSensors_c {
public:
  float readings[NUM_SENSORS];

  float minimum[NUM_SENSORS];
  float maximum[NUM_SENSORS];
  float scaling[NUM_SENSORS];
  float range[NUM_SENSORS];
  float calibrated[NUM_SENSORS];

  ReadState state[NUM_SENSORS];
  unsigned long charge_start_us;
  unsigned long discharge_start_us[NUM_SENSORS];
  bool sample_ready = false;

  unsigned long timeout_us = 000;
  unsigned long charge_us = 10;

  LineSensors_c() {}

  // -----------------------------
  // ADC INITIALISATION
  // -----------------------------
  void initialiseForADC() {
    pinMode(EMIT_PIN, INPUT);   // follower mode: emitters OFF

    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      pinMode(sensor_pins[sensor], INPUT_PULLUP);
    }
  }

  // -----------------------------
  // ADC READ
  // -----------------------------
  void readSensorsADC() {
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      readings[sensor] = analogRead(sensor_pins[sensor]);
    }
  }

  // -----------------------------
  // ADC CALIBRATED READ
  // -----------------------------
  void calcCalibratedADC() {
    readSensorsADC();

    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      if (range[sensor] <= 0) {
        calibrated[sensor] = 0;
      } else {
        calibrated[sensor] = (readings[sensor] - minimum[sensor]) / range[sensor];
      }
    }
  }

  // -----------------------------
  // DIGITAL INITIALISATION
  // -----------------------------
  void initialiseForDigital() {
    pinMode(EMIT_PIN, INPUT);   // follower mode: emitters OFF

    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      pinMode(sensor_pins[sensor], INPUT);
    }
  }

  // -----------------------------
  // DIGITAL READ OF ONE SENSOR
  // Returns elapsed discharge time in microseconds
  // Smaller time usually means stronger IR
  // -----------------------------
  unsigned long readSensorDigital(int which_sensor) {
    if (which_sensor < 0 || which_sensor >= NUM_SENSORS) {
      return timeout_us;
    }

    int pin = sensor_pins[which_sensor];

    pinMode(EMIT_PIN, INPUT);   // follower mode: emitters OFF

    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delayMicroseconds(charge_us);

    pinMode(pin, INPUT);

    unsigned long start_time = micros();

    while (digitalRead(pin) == HIGH) {
      if (micros() - start_time > timeout_us) {
        return timeout_us;
      }
    }

    return micros() - start_time;
  }

  // -----------------------------
  // DIGITAL READ OF ALL SENSORS
  // -----------------------------
  void readSensorsDigital() {
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      readings[sensor] = readSensorDigital(sensor);
    }
  }

  // -----------------------------
  // DIGITAL CALIBRATED READ
  // -----------------------------
  void calcCalibratedDigital() {
    readSensorsDigital();

    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      if (range[sensor] <= 0) {
        calibrated[sensor] = 0;
      } else {
        calibrated[sensor] = (readings[sensor] - minimum[sensor]) / range[sensor];
      }
    }
  }

  void startDigitalReadAll() {
    pinMode(EMIT_PIN, INPUT);

    charge_start_us = micros();
    sample_ready = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
      pinMode(sensor_pins[i], OUTPUT);
      digitalWrite(sensor_pins[i], HIGH);
      state[i] = CHARGING;
    }
  }

  void serviceDigitalRead() {
    unsigned long now = micros();

    if (now - charge_start_us >= charge_us) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (state[i] == CHARGING) {
          pinMode(sensor_pins[i], INPUT);
          discharge_start_us[i] = micros();
          state[i] = DISCHARGING;
        }
      }
    }

    bool all_done = true;

    for (int i = 0; i < NUM_SENSORS; i++) {
      if (state[i] == DISCHARGING) {
        if (digitalRead(sensor_pins[i]) == LOW) {
          readings[i] = micros() - discharge_start_us[i];
          state[i] = DONE;
        } else if (micros() - discharge_start_us[i] >= timeout_us) {
          readings[i] = timeout_us;
          state[i] = DONE;
        }
      }

      if (state[i] != DONE) {
        all_done = false;
      }
    }

    if (all_done) {
      sample_ready = true;
    }
  }

  void calcCalibratedFromCurrentReadings() {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (range[i] <= 0) {
        calibrated[i] = 0;
      } else {
        calibrated[i] = (readings[i] - minimum[i]) / range[i];
      }

      if (calibrated[i] < 0) calibrated[i] = 0;
      if (calibrated[i] > 1) calibrated[i] = 1;
    }
  }

  void updateCalibrationFromReadings() {
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      if (readings[sensor] < minimum[sensor]) minimum[sensor] = readings[sensor];
      if (readings[sensor] > maximum[sensor]) maximum[sensor] = readings[sensor];
    }
  }

  void finaliseCalibration() {
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      range[sensor] = maximum[sensor] - minimum[sensor];
      if (range[sensor] <= 0) range[sensor] = 1;
    }
  }

  bool sampleReady() {
    return sample_ready;
  }

  void clearSampleReady() {
    sample_ready = false;
  }
};

#endif
