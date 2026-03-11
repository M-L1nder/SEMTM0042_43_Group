#include "PID.h"
#include "Motors.h"
#include "BumpSensors.h"
#include "Encoders.h"
#include <math.h>

Motors_c motors;
PID_c left_pid;
PID_c right_pid;
BumpSensors_c bump_sensors;

// -------------------- PINS --------------------
#define BUTTON_A_PIN 14

// -------------------- TIMING --------------------
unsigned long speed_est_ts;
#define SPEED_EST_MS 10

// -------------------- SPEED ESTIMATION --------------------
long last_e0;
long last_e1;
float last_speed_e0;
float last_speed_e1;
float right_speed;
float left_speed;
#define alpha 0.2

// -------------------- FOLLOWER STATE --------------------
enum FollowerState {
  WAITING_FOR_BEACON_TO_DISAPPEAR,
  WAITING_FOR_BEACON,
  FOLLOWING
};

FollowerState follower_state = WAITING_FOR_BEACON_TO_DISAPPEAR;
unsigned long beacon_lost_ms = 0;

// -------------------- BASELINE + TARGET --------------------
float bump_baseline[BUMP_NUM_SENSORS];
float target_signal = 0.20;
bool target_captured = false;
bool lastButtonAState = HIGH;

// -------------------- FILTERED SIGNAL --------------------
float L_filt = 0.0;
float R_filt = 0.0;

// -------------------- TUNING --------------------
// Flip this if the follower drives the wrong direction.
const float FOLLOW_SIGN = 1.0;

// Signal thresholds
const float BEACON_PRESENT_THRESH = 0.03;
const unsigned long LOST_TIMEOUT_MS = 200;

// Follow speed control
const float BASE_SPEED   = 0.50;   // should be near leader speed magnitude
const float K_DIST       = 0.90;   // correction around base speed
const float SEARCH_SPEED = 0.30;   // used briefly if signal drops
const float MAX_SPEED    = 0.85;
const float MIN_SPEED    = 0.00;   // keep non-negative for now

// Signal filtering
const float FILTER_BETA = 0.35;

void setup() {
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  last_e0 = count_e0;
  last_e1 = count_e1;
  last_speed_e0 = 0.0;
  last_speed_e1 = 0.0;

  left_pid.initialise(15.0, 0.1, 0.0);
  right_pid.initialise(15.0, 0.1, 0.0);
  left_pid.reset();
  right_pid.reset();

  bump_sensors.initialiseForDigital();
  bump_sensors.timeout_us = 2500;

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  Serial.println("Measure baseline: keep leader beacon OFF");
  measureBumpBaseline();

  Serial.println("Place leader at desired distance, turn beacon ON, then press button A");

  while (!target_captured) {
    bool currentButton = digitalRead(BUTTON_A_PIN);

    if (lastButtonAState == HIGH && currentButton == LOW) {
      delay(20);
      if (digitalRead(BUTTON_A_PIN) == LOW) {
        captureTargetSignal();
      }
    }

    lastButtonAState = currentButton;
  }

  // After capture, wait for beacon to disappear before arming for the real run.
  follower_state = WAITING_FOR_BEACON_TO_DISAPPEAR;

  L_filt = 0.0;
  R_filt = 0.0;

  bump_sensors.startDigitalReadAll();
  speed_est_ts = millis();

  Serial.println("Lsig,Rsig,mean,left_speed,right_speed,state");
}

void loop() {
  // Keep non-blocking bump sensor read progressing
  bump_sensors.serviceDigitalRead();

  // Update wheel speed estimate
  unsigned long now = millis();
  if (now - speed_est_ts >= SPEED_EST_MS) {
    unsigned long elapsed = now - speed_est_ts;
    speedcalc(elapsed);
    speed_est_ts = now;
  }

  // Only act when a fresh paired bump sample is ready
  if (bump_sensors.sampleReady()) {
    float L_raw = bump_sensors.readings[0];
    float R_raw = bump_sensors.readings[1];

    float L_sig = readingToSignal(L_raw, bump_baseline[0]);
    float R_sig = readingToSignal(R_raw, bump_baseline[1]);

    // Low-pass filter
    L_filt = (1.0 - FILTER_BETA) * L_filt + FILTER_BETA * L_sig;
    R_filt = (1.0 - FILTER_BETA) * R_filt + FILTER_BETA * R_sig;

    float mean_signal = 0.5 * (L_filt + R_filt);
    bool beacon_present = (mean_signal > BEACON_PRESENT_THRESH);

    if (follower_state == WAITING_FOR_BEACON_TO_DISAPPEAR) {
      stoprobot();

      // After target capture, wait until the beacon is turned off.
      if (!beacon_present) {
        follower_state = WAITING_FOR_BEACON;
      }
    }
    else if (follower_state == WAITING_FOR_BEACON) {
      stoprobot();

      // Start only when beacon appears again for the actual run.
      if (beacon_present) {
        left_pid.reset();
        right_pid.reset();
        follower_state = FOLLOWING;
        beacon_lost_ms = 0;
      }
    }
    else if (follower_state == FOLLOWING) {
      if (beacon_present) {
        beacon_lost_ms = 0;
        followDistanceOnly(mean_signal);
      } else {
        if (beacon_lost_ms == 0) {
          beacon_lost_ms = millis();
        }

        if (millis() - beacon_lost_ms > LOST_TIMEOUT_MS) {
          stoprobot();
          follower_state = WAITING_FOR_BEACON;
        } else {
          // brief drop-out: keep creeping forward
          driveAtTargetSpeed(SEARCH_SPEED);
        }
      }
    }

    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 100) {
      lastLog = millis();

      Serial.print(L_filt, 3);
      Serial.print(",");
      Serial.print(R_filt, 3);
      Serial.print(",");
      Serial.print(mean_signal, 3);
      Serial.print(",");
      Serial.print(target_signal, 3);
      Serial.print(",");
      Serial.print(left_speed, 3);
      Serial.print(",");
      Serial.print(right_speed, 3);
      Serial.print(",");
      if (follower_state == WAITING_FOR_BEACON_TO_DISAPPEAR) Serial.println("DISARM");
      else if (follower_state == WAITING_FOR_BEACON) Serial.println("WAIT");
      else Serial.println("FOLLOW");
    }

    bump_sensors.clearSampleReady();
    bump_sensors.startDigitalReadAll();
  }
}

// -------------------- DISTANCE CONTROL ONLY --------------------
void followDistanceOnly(float mean_signal)
{
  float target_speed = 0.0;

  // signal missing / weak
  if (mean_signal < BEACON_PRESENT_THRESH) {
    target_speed = SEARCH_SPEED;
  }
  else {
    float error = target_signal - mean_signal;

    // If we are close enough, stop
    if (fabs(error) < 0.03) {
      target_speed = 0.0;
    }
    // If beacon is stronger than target, we are too close -> stop
    else if (mean_signal > target_signal) {
      target_speed = 0.0;
    }
    // If beacon is weaker than target, we are too far -> move forward
    else {
      target_speed = BASE_SPEED + K_DIST * error;
    }
  }

  if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;
  if (target_speed < MIN_SPEED) target_speed = MIN_SPEED;

  driveAtTargetSpeed(target_speed);
}
// -------------------- WHEEL COMMAND --------------------
void driveAtTargetSpeed(float target_speed)
{
  target_speed *= FOLLOW_SIGN;

  float l_pwm = left_pid.update(target_speed, left_speed);
  float r_pwm = right_pid.update(target_speed, right_speed);

  motors.setPWM(l_pwm, r_pwm);
}

// -------------------- RAW READING -> SIGNAL --------------------
// More IR -> smaller discharge time -> bigger signal
float readingToSignal(float reading, float baseline)
{
  if (baseline <= 1.0) return 0.0;

  float x = (baseline - reading) / baseline;

  if (x < 0.0) x = 0.0;
  if (x > 1.0) x = 1.0;

  return x;
}

// -------------------- BASELINE MEASUREMENT --------------------
// Measure ambient/background only, with leader beacon OFF
void measureBumpBaseline()
{
  const int N = 80;

  for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
    bump_baseline[s] = 0.0;
  }

  for (int i = 0; i < N; i++) {
    bump_sensors.readSensorsDigital();

    for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
      bump_baseline[s] += bump_sensors.readings[s];
    }

    delay(5);
  }

  for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
    bump_baseline[s] /= N;
  }

  Serial.print("Baseline L = ");
  Serial.print(bump_baseline[0]);
  Serial.print("  R = ");
  Serial.println(bump_baseline[1]);
}

// -------------------- TARGET CAPTURE --------------------
// Measure beacon signal with leader beacon ON at desired distance
void captureTargetSignal()
{
  const int N = 60;
  float sumL = 0.0;
  float sumR = 0.0;

  Serial.println("Capturing target signal...");

  for (int i = 0; i < N; i++) {
    bump_sensors.readSensorsDigital();

    float L_raw = bump_sensors.readings[0];
    float R_raw = bump_sensors.readings[1];

    float L_sig = readingToSignal(L_raw, bump_baseline[0]);
    float R_sig = readingToSignal(R_raw, bump_baseline[1]);

    sumL += L_sig;
    sumR += R_sig;

    delay(5);
  }

  float meanL = sumL / N;
  float meanR = sumR / N;
  target_signal = 0.5 * (meanL + meanR);

  Serial.print("Captured target_signal = ");
  Serial.println(target_signal, 3);

  target_captured = true;
}

// -------------------- STOP --------------------
void stoprobot()
{
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

// -------------------- SPEED ESTIMATION --------------------
void speedcalc(unsigned long elapsed_time)
{
  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;

  float speed_e0 = float(delta_e0) / float(elapsed_time);
  float speed_e1 = float(delta_e1) / float(elapsed_time);

  // If one wheel sign is wrong, flip one of these:
  // float speed_e1 = -float(delta_e1) / float(elapsed_time);

  right_speed = alpha * speed_e0 + (1 - alpha) * last_speed_e0;
  left_speed  = alpha * speed_e1 + (1 - alpha) * last_speed_e1;

  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
  last_e0 = count_e0;
  last_e1 = count_e1;
}
