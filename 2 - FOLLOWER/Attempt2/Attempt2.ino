#include "PID.h"
#include "Motors.h"
#include "BumpSensors.h"
#include "Encoders.h"

Motors_c motors;
PID_c left_pid;
PID_c right_pid;
BumpSensors_c bump_sensors;

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
  WAITING_FOR_BEACON,
  FOLLOWING
};

FollowerState follower_state = WAITING_FOR_BEACON;
unsigned long beacon_lost_ms = 0;

// -------------------- BASELINE --------------------
float bump_baseline[BUMP_NUM_SENSORS];

// -------------------- FILTERED SIGNAL --------------------
float L_filt = 0.0;
float R_filt = 0.0;

// -------------------- TUNING --------------------
// If follower drives the wrong direction, flip this between +1 and -1
const float FOLLOW_SIGN = 1.0;

// Signal thresholds
const float BEACON_PRESENT_THRESH = 0.03;   // signal must exceed this to count as "seen"
const float TARGET_SIGNAL = 0.20;           // target mean signal at desired distance
const float STOP_NEAR_THRESH = 0.35;        // if signal stronger than this, stop
const unsigned long LOST_TIMEOUT_MS = 200;  // stop if beacon gone this long

// Follow speed control
const float BASE_SPEED = 0.50;   // should be similar to leader speed magnitude
const float K_DIST = 0.90;       // correction around base speed
const float SEARCH_SPEED = 0.30; // brief forward motion if beacon weak
const float MAX_SPEED = 0.85;
const float MIN_SPEED = 0.00;    // keep non-negative for now

// Signal filtering
const float FILTER_BETA = 0.35;

// -------------------- SETUP --------------------
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

  // Faster timeout usually helps response
  bump_sensors.timeout_us = 2500;

  // IMPORTANT:
  // Leader beacon should be OFF or far away during this baseline measurement
  measureBumpBaseline();

  bump_sensors.startDigitalReadAll();
  speed_est_ts = millis();

  Serial.println("Lsig,Rsig,mean,left_speed,right_speed,state");
}

// -------------------- MAIN LOOP --------------------
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

    if (follower_state == WAITING_FOR_BEACON) {
      stoprobot();

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

        // small grace period before stopping
        if (millis() - beacon_lost_ms > LOST_TIMEOUT_MS) {
          stoprobot();
          follower_state = WAITING_FOR_BEACON;
        } else {
          // brief weak signal: keep moving forward gently
          driveAtTargetSpeed(SEARCH_SPEED);
        }
      }
    }

    // Log at a lower rate so serial doesn’t slow control
    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 100) {
      lastLog = millis();

      Serial.print(L_filt, 3);
      Serial.print(",");
      Serial.print(R_filt, 3);
      Serial.print(",");
      Serial.print(mean_signal, 3);
      Serial.print(",");
      Serial.print(left_speed, 3);
      Serial.print(",");
      Serial.print(right_speed, 3);
      Serial.print(",");
      Serial.println(follower_state == WAITING_FOR_BEACON ? "WAIT" : "FOLLOW");
    }

    bump_sensors.clearSampleReady();
    bump_sensors.startDigitalReadAll();
  }
}

// -------------------- DISTANCE CONTROL ONLY --------------------
void followDistanceOnly(float mean_signal)
{
  float target_speed = 0.0;

  // If very close, stop
  if (mean_signal > STOP_NEAR_THRESH) {
    target_speed = 0.0;
  } 
  else {
    // If signal weaker than target, we are too far away
    float error = TARGET_SIGNAL - mean_signal;

    if (abs(error) < 0.02) {
      error = 0.0;
    }

    target_speed = BASE_SPEED + K_DIST * error;
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
