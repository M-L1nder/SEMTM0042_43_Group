// Class for Bumpsesnors on pins A6 and 5

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _BUMPSENSORS_H
#define _BUMPSENSORS_H

// We will use all 5 line sensors (DN1 - 5)
// and so define a constant here, rather than
// type '5' in lots of places.
#define BUMP_NUM_SENSORS 2


// Pin definitions
// This time, we will use an array to store the
// pin definitions.  This is a bit like a list.
// This way, we can either loop through the
// list automatically, or we can ask for a pin
// by indexing, e.g. sensor_pins[0] is A11,
// sensors_pins[1] is A0.
const int bump_sensor_pins[ BUMP_NUM_SENSORS ] = { A6, 5};

// This is the pin used to turn on the infra-
// red LEDs.
#define EMIT_PIN   11

enum ReadState { IDLE, CHARGING, DISCHARGING, DONE };

ReadState state[BUMP_NUM_SENSORS];
unsigned long charge_start_us;
unsigned long discharge_start_us[BUMP_NUM_SENSORS];
bool sample_ready = false;

unsigned long timeout_us = 3000;
unsigned long charge_us = 10;




// Class to operate the linesensors.
class BumpSensors_c {

  public:

    // Raw Readings [0:1]
    float readings[ BUMP_NUM_SENSORS ];

    // Calibration values
    float minimum[ BUMP_NUM_SENSORS ];
    float maximum[ BUMP_NUM_SENSORS ];
    float scaling[ BUMP_NUM_SENSORS ];
    float range[ BUMP_NUM_SENSORS ];

    // Calibrated values
    float bumpcalibrated[ BUMP_NUM_SENSORS ];

    // Timeout for digital discharge-time read
    unsigned long timeout_us = 6000;

    // Constructor, must exist.
    BumpSensors_c() {
      // leave this empty
    }

    // -----------------------------
    // ADC INITIALISATION
    // -----------------------------
    void initialiseForADC() {

      // Follower mode: emitters OFF
      pinMode(EMIT_PIN, INPUT);

      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
        pinMode(bump_sensor_pins[sensor], INPUT);
      }
    }

    // -----------------------------
    // ADC READ
    // -----------------------------
    void readSensorsADC() {
      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
        readings[sensor] = analogRead(bump_sensor_pins[sensor]);
      }
    }

    // -----------------------------
    // ADC CALIBRATED READ
    // -----------------------------
    void calcCalibratedADC() {

      readSensorsADC();

      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
        if (range[sensor] <= 0) {
          bumpcalibrated[sensor] = 0;
        } else {
          bumpcalibrated[sensor] = (readings[sensor] - minimum[sensor]) / range[sensor];
        }
      }
    }

    // -----------------------------
    // DIGITAL INITIALISATION
    // -----------------------------
    void initialiseForDigital() {

      // Follower mode: emitters OFF
      pinMode(EMIT_PIN, INPUT);

      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
        pinMode(bump_sensor_pins[sensor], INPUT);
      }
    }

    // -----------------------------
    // DIGITAL READ OF ONE SENSOR
    // Returns elapsed discharge time in microseconds
    // Smaller time usually means stronger IR
    // -----------------------------
    unsigned long readSensorDigital(int which_sensor) {

      if (which_sensor < 0 || which_sensor >= BUMP_NUM_SENSORS) {
        return timeout_us;
      }

      int pin = bump_sensor_pins[which_sensor];

      // Follower mode: emitters OFF so we measure external IR beacon
      pinMode(EMIT_PIN, INPUT);

      // Charge capacitor/node
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      delayMicroseconds(10);

      // Let it discharge
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
      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
        readings[sensor] = readSensorDigital(sensor);
      }
    }


    // -----------------------------
    // DIGITAL CALIBRATED READ
    // -----------------------------
    void calcCalibratedDigital() {

      readSensorsDigital();

      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
        if (range[sensor] <= 0) {
          bumpcalibrated[sensor] = 0;
        } else {
          bumpcalibrated[sensor] = (readings[sensor] - minimum[sensor]) / range[sensor];
        }
      }
    }


    void startDigitalReadAll() {
      pinMode(EMIT_PIN, INPUT);   // follower emitters OFF

      charge_start_us = micros();
      sample_ready = false;

      for (int i = 0; i < BUMP_NUM_SENSORS; i++) {
        pinMode(bump_sensor_pins[i], OUTPUT);
        digitalWrite(bump_sensor_pins[i], HIGH);
        state[i] = CHARGING;
      }
    }

    void serviceDigitalRead() {
      unsigned long now = micros();

      // After charge time, switch all charging sensors to input
      if (now - charge_start_us >= charge_us) {
        for (int i = 0; i < BUMP_NUM_SENSORS; i++) {
          if (state[i] == CHARGING) {
            pinMode(bump_sensor_pins[i], INPUT);
            discharge_start_us[i] = micros();
            state[i] = DISCHARGING;
          }
        }
      }

      bool all_done = true;

      for (int i = 0; i < BUMP_NUM_SENSORS; i++) {
        if (state[i] == DISCHARGING) {
          if (digitalRead(bump_sensor_pins[i]) == LOW) {
            readings[i] = micros() - discharge_start_us[i];
            state[i] = DONE;
          }
          else if (micros() - discharge_start_us[i] >= timeout_us) {
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
      for (int i = 0; i < BUMP_NUM_SENSORS; i++) {
        bumpcalibrated[i] = (readings[i] - minimum[i]) / range[i];

        if (bumpcalibrated[i] < 0) bumpcalibrated[i] = 0;
        if (bumpcalibrated[i] > 1) bumpcalibrated[i] = 1;
      }
    }
    // -----------------------------
    // Helper: update min/max from latest readings
    // Call this inside your calibration routine
    // -----------------------------
    void updateCalibrationFromReadings() {
      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
        if (readings[sensor] < minimum[sensor]) minimum[sensor] = readings[sensor];
        if (readings[sensor] > maximum[sensor]) maximum[sensor] = readings[sensor];
      }
    }

    // -----------------------------
    // Helper: calculate ranges after calibration
    // -----------------------------
    void finaliseCalibration() {
      for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
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
}; // End of bumpsensor class defintion



#endif
