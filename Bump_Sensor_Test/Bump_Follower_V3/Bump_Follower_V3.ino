//Tried to do offset, very very tempremental


//#include "Motors.h"
//#include "BumpSensors.h"
//#include "PID.h"
//#include "Encoders.h"
//#include "oled.h"
//OLED_c display(1, 30, 0, 17, 13);   // clk, mosi, rst, dc, cs
//
//unsigned long oled_ts = 0;
//#define OLED_UPDATE_MS 200   // fairly responsive without flicker
//
//
//Motors_c motors;
//BumpSensors_c bump_sensors;
//PID_c left_pid;
//PID_c right_pid;
//
//// -------------------- BUTTON --------------------
//const int BUTTON_A_PIN = 14;
//
//// -------------------- TIMING --------------------
//uint32_t lastLoop = 0;
//const uint32_t LOOP_TIME_MS = 20;
//
//// speed estimation timing
//unsigned long speed_est_ts = 0;
//const uint32_t SPEED_EST_MS = 10;
//const float ALPHA = 0.2f;
//
//// -------------------- SPEED ESTIMATION --------------------
//long last_e0 = 0;
//long last_e1 = 0;
//
//float right_speed = 0.0f;
//float left_speed = 0.0f;
//float last_speed_e0 = 0.0f;
//float last_speed_e1 = 0.0f;
//
//// -------------------- BUMP SIGNALS --------------------
//float L = 0.0f;
//float R = 0.0f;
//float total = 0.0f;
//
//// baseline raw discharge times (beacon OFF)
//float bump_baseline[BUMP_NUM_SENSORS];
//
//// -------------------- TARGET SAMPLE --------------------
//float targetTotal = 0.0f;
//float targetDiff = 0.0f;
//
//float lowerThreshold = 0.0f;
//float upperThreshold = 0.0f;
//
//float targetSampleSumTotal = 0.0f;
//float targetSampleSumDiff = 0.0f;
//int targetSampleCount = 0;
//
//float minTotalSeen = 100000.0f;
//float maxTotalSeen = -100000.0f;
//
//// -------------------- TUNING --------------------
//// target capture time
//const uint32_t TARGET_SAMPLE_TIME_MS = 2000;
//
//// after target capture, wait for beacon to disappear, then reappear for run start
//const float BEACON_PRESENT_TOTAL = 15.0f;   // tune from serial logs
//const uint32_t LOST_TIMEOUT_MS = 200;
//
//// drive control
//const float MAX_FWD_SPEED_DEMAND = 0.45f;  // should be a bit above leader speed magnitude
//const float SPEED_GAIN = 0.015f;           // converts signal deficit -> speed demand
//const float TARGET_DEADBAND = 2.0f;        // band around target before moving
//
//// threshold band built from observed target variation
//const float BAND_MARGIN_FRACTION = 0.20f;
//const float MIN_BAND_MARGIN = 4.0f;
//
//// pwm limits / floor
//const int PWM_MAX_ABS = 60;
//const int PWM_FLOOR_FWD = 18;
//
//// pid gains
//const float DRIVE_KP = 15.0f;
//const float DRIVE_KI = 0.1f;
//const float DRIVE_KD = 0.0f;
//
//// if follower drives backwards instead of forwards, flip this
//const float FOLLOW_SIGN = 1.0f;
//// turn / offset control
//const float TURN_GAIN = 0.15f;          // converts diff error -> wheel speed demand
//const float MAX_TURN_SPEED_DEMAND = 0.20f;
//const float DIFF_DEADBAND = 5.0f;        // deadband on (R-L) error
//const float DIFF_BIG = 10.0f;            // if diff error is large, reduce forward drive
//
//// -------------------- STATE --------------------
//enum RobotState {
//  WAITING_FOR_TARGET_BUTTON,
//  SAMPLING_TARGET,
//  WAITING_FOR_BEACON_OFF,
//  WAITING_FOR_BEACON_ON,
//  FOLLOWING
//};
//
//RobotState robotstate = WAITING_FOR_TARGET_BUTTON;
//
//bool lastButtonState = HIGH;
//uint32_t targetSampleStart = 0;
//uint32_t beaconLostStart = 0;
//float smoothedDrive = 0.0f;
//
//// -------------------- HELPERS --------------------
//const char* getStateName(RobotState s)
//{
//  switch (s) {
//    case WAITING_FOR_TARGET_BUTTON: return "WAIT_TGT";
//    case SAMPLING_TARGET:           return "SAMPLE";
//    case WAITING_FOR_BEACON_OFF:    return "WAIT_OFF";
//    case WAITING_FOR_BEACON_ON:     return "WAIT_ON";
//    case FOLLOWING:                 return "FOLLOW";
//    default:                        return "UNKNOWN";
//  }
//}
//
//void drawFollowerOLED()
//{
//  display.clear();
//
//  display.gotoXY(0, 0);
//  display.print(getStateName(robotstate));
//
//  display.gotoXY(0, 1);
//  display.print("T:");
//  display.print(total, 1);
//  display.print("/");
//  display.print(targetTotal, 1);
//}
//
//float clampFloat(float x, float lo, float hi) {
//  if (x < lo) return lo;
//  if (x > hi) return hi;
//  return x;
//}
//
//int clampInt(int x, int lo, int hi) {
//  if (x < lo) return lo;
//  if (x > hi) return hi;
//  return x;
//}
//
//int applyPwmFloor(int pwm, float demand_speed, int floorVal) {
//  if (fabsf(demand_speed) < 1e-6f) return 0;
//  if (abs(pwm) < floorVal) {
//    pwm = (demand_speed > 0.0f) ? floorVal : -floorVal;
//  }
//  return pwm;
//}
//
//void stopRobot() {
//  motors.setPWM(0, 0);
//  left_pid.reset();
//  right_pid.reset();
//}
//
//void speedcalc(unsigned long elapsed_time) {
//  long delta_e0 = count_e0 - last_e0;
//  long delta_e1 = count_e1 - last_e1;
//
//  last_e0 = count_e0;
//  last_e1 = count_e1;
//
//  float speed_e0 = (float)delta_e0 / (float)elapsed_time;
//  float speed_e1 = (float)delta_e1 / (float)elapsed_time;
//
//  right_speed = ALPHA * speed_e0 + (1.0f - ALPHA) * last_speed_e0;
//  left_speed  = ALPHA * speed_e1 + (1.0f - ALPHA) * last_speed_e1;
//
//  last_speed_e0 = right_speed;
//  last_speed_e1 = left_speed;
//}
//
//void updateWheelSpeedsIfNeeded() {
//  unsigned long elapsed_time = millis() - speed_est_ts;
//  if (elapsed_time >= SPEED_EST_MS) {
//    speedcalc(elapsed_time);
//    speed_est_ts = millis();
//  }
//}
//
//// Convert raw discharge time into a positive signal.
//// More IR -> smaller raw reading -> bigger signal.
//float readingToSignal(float reading, float baseline) {
//  if (baseline <= 1.0f) return 0.0f;
//
//  float x = 100.0f * (baseline - reading) / baseline;
//
//  if (x < 0.0f) x = 0.0f;
//  if (x > 100.0f) x = 100.0f;
//
//  return x;
//}
//
//void updateBumpSignals() {
//  bump_sensors.readSensorsDigital();
//
//  L = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
//  R = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);
//
//  total = L + R;
//}
//
//void measureBumpBaseline() {
//  const int N = 80;
//
//  for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
//    bump_baseline[s] = 0.0f;
//  }
//
//  for (int i = 0; i < N; i++) {
//    bump_sensors.readSensorsDigital();
//
//    for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
//      bump_baseline[s] += bump_sensors.readings[s];
//    }
//
//    delay(5);
//  }
//
//  for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
//    bump_baseline[s] /= N;
//  }
//
//  Serial.print("Baseline L raw = ");
//  Serial.print(bump_baseline[0]);
//  Serial.print("  R raw = ");
//  Serial.println(bump_baseline[1]);
//}
//
//void startTargetSampling() {
//  targetSampleStart = millis();
//  targetSampleSumTotal = 0.0f;
//  targetSampleSumDiff = 0.0f;
//  targetSampleCount = 0;
//  minTotalSeen = 100000.0f;
//  maxTotalSeen = -100000.0f;
//}
//void finishTargetSampling() {
//  if (targetSampleCount < 1) targetSampleCount = 1;
//
//  targetTotal = targetSampleSumTotal / (float)targetSampleCount;
//  targetDiff  = targetSampleSumDiff  / (float)targetSampleCount;
//
//  float observedSpread = maxTotalSeen - minTotalSeen;
//  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
//  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;
//
//  lowerThreshold = targetTotal - bandMargin;
//  upperThreshold = targetTotal + bandMargin;
//
//  //  Serial.println("target sampling done");
//  //  Serial.print("targetTotal,"); Serial.println(targetTotal);
//  //  Serial.print("targetDiff,");  Serial.println(targetDiff);
//  //  Serial.print("lowerThreshold,"); Serial.println(lowerThreshold);
//  //  Serial.print("upperThreshold,"); Serial.println(upperThreshold);
//}
//void driveAtDemands(float leftDemand, float rightDemand) {
//  leftDemand  = clampFloat(leftDemand,  -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);
//  rightDemand = clampFloat(rightDemand, -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);
//
//  leftDemand  *= FOLLOW_SIGN;
//  rightDemand *= FOLLOW_SIGN;
//
//  float l_pwm_f = left_pid.update(leftDemand, left_speed);
//  float r_pwm_f = right_pid.update(rightDemand, right_speed);
//
//  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
//  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
//
//  l_pwm = applyPwmFloor(l_pwm, leftDemand, PWM_FLOOR_FWD);
//  r_pwm = applyPwmFloor(r_pwm, rightDemand, PWM_FLOOR_FWD);
//
//  motors.setPWM(l_pwm, r_pwm);
//}
//
//void driveAtDemand(float demandSpeed) {
//  driveAtDemands(demandSpeed, demandSpeed);
//}
//
//// -------------------- SETUP --------------------
//void setup() {
//  Serial.begin(115200);
//
//  motors.initialise();
//  bump_sensors.initialiseForDigital();
//  setupEncoder0();
//  setupEncoder1();
//
//  // Faster timeout helps responsiveness
//  bump_sensors.timeout_us = 2500;
//
//  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
//
//  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
//  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
//  left_pid.reset();
//  right_pid.reset();
//
//  last_e0 = count_e0;
//  last_e1 = count_e1;
//  speed_est_ts = millis();
//
//  // Baseline must be measured with beacon OFF
//  Serial.println("Measure baseline: keep leader beacon OFF");
//  measureBumpBaseline();
//
//  Serial.println("Place leader at desired distance, beacon ON, then press follower button A");
//
//  robotstate = WAITING_FOR_TARGET_BUTTON;
//  drawFollowerOLED();
//  oled_ts = millis();
//  lastLoop = millis();
//}
//
//// -------------------- LOOP --------------------
//void loop() {
//  updateWheelSpeedsIfNeeded();
//
//  if (millis() - lastLoop < LOOP_TIME_MS) return;
//  lastLoop += LOOP_TIME_MS;
//
//  // Update bump signals in all states after baseline
//  updateBumpSignals();
//
//  bool buttonState = digitalRead(BUTTON_A_PIN);
//
//  if (robotstate == WAITING_FOR_TARGET_BUTTON) {
//    stopRobot();
//
//    // press follower button to begin target capture
//    if (lastButtonState == HIGH && buttonState == LOW) {
//      startTargetSampling();
//      robotstate = SAMPLING_TARGET;
//    }
//  }
//  else if (robotstate == SAMPLING_TARGET) {
//    stopRobot();
//
//    // accumulate only while beacon is actually present
//    if (total > BEACON_PRESENT_TOTAL) {
//      float diff = R - L;
//
//      targetSampleSumTotal += total;
//      targetSampleSumDiff += diff;
//      targetSampleCount++;
//
//      if (total < minTotalSeen) minTotalSeen = total;
//      if (total > maxTotalSeen) maxTotalSeen = total;
//    }
//
//    if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
//      finishTargetSampling();
//      robotstate = WAITING_FOR_BEACON_OFF;
//    }
//  }
//  else if (robotstate == WAITING_FOR_BEACON_OFF) {
//    stopRobot();
//
//    // after target capture, wait until beacon is turned off
//    if (total < BEACON_PRESENT_TOTAL) {
//      robotstate = WAITING_FOR_BEACON_ON;
//    }
//  }
//  else if (robotstate == WAITING_FOR_BEACON_ON) {
//    stopRobot();
//
//    // start following only when beacon reappears for the real run
//    if (total > BEACON_PRESENT_TOTAL) {
//      left_pid.reset();
//      right_pid.reset();
//      smoothedDrive = 0.0f;
//      beaconLostStart = 0;
//      robotstate = FOLLOWING;
//    }
//  }
//  else if (robotstate == FOLLOWING) {
//    if (total < BEACON_PRESENT_TOTAL) {
//      // beacon lost: give it a short grace period, then stop
//      if (beaconLostStart == 0) beaconLostStart = millis();
//
//      if (millis() - beaconLostStart > LOST_TIMEOUT_MS) {
//        stopRobot();
//        robotstate = WAITING_FOR_BEACON_ON;
//      } else {
//        driveAtDemand(0.0f);
//      }
//    }
//    else {
//      beaconLostStart = 0;
//
//      float diff = R - L;
//
//      // ---------- total control (distance / fore-aft) ----------
//      float totalError = (targetTotal - TARGET_DEADBAND) - total;
//      float desiredDrive = 0.0f;
//
//      if (totalError > 0.0f) {
//        desiredDrive = SPEED_GAIN * totalError;
//      } else {
//        desiredDrive = 0.0f;
//      }
//
//      desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);
//
//      // ---------- diff control (offset / bearing) ----------
//      float diffError = targetDiff - diff;
//
//      if (fabs(diffError) < DIFF_DEADBAND) {
//        diffError = 0.0f;
//      }
//
//      float desiredTurn = TURN_GAIN * diffError;
//      desiredTurn = clampFloat(desiredTurn, -MAX_TURN_SPEED_DEMAND, MAX_TURN_SPEED_DEMAND);
//
//      // If the offset error is large, reduce forward motion so it doesn't snake badly
//      if (fabs(diffError) > DIFF_BIG) {
//        desiredDrive *= 0.3f;
//      }
//
//      // Combine
//      float leftDemand  = desiredDrive - desiredTurn;
//      float rightDemand = desiredDrive + desiredTurn;
//
//      // store for debug
//      smoothedDrive = desiredDrive;
//
//      driveAtDemands(leftDemand, rightDemand);
//    }
//  }
//
//  lastButtonState = buttonState;
//
//  Serial.print(total);
//  Serial.print(",");
//  Serial.print(targetTotal);
//  Serial.print(",");
//  Serial.print(targetDiff);
//  Serial.print(",");
//  Serial.print(L);
//  Serial.print(",");
//  Serial.print(R);
//  Serial.print(",");
//  Serial.print(R - L);
//  Serial.print(",");
//  Serial.print(smoothedDrive);
//  Serial.print(",");
//  Serial.print(left_speed);
//  Serial.print(",");
//  Serial.print(right_speed);
//  Serial.print(",");
//  Serial.println((int)robotstate);
//  if (millis() - oled_ts >= OLED_UPDATE_MS) {
//    oled_ts = millis();
//    drawFollowerOLED();
//  }
//}


#include "Motors.h"
#include "BumpSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "oled.h"
#include <math.h>

OLED_c display(1, 30, 0, 17, 13);   // clk, mosi, rst, dc, cs

unsigned long oled_ts = 0;
#define OLED_UPDATE_MS 200

Motors_c motors;
BumpSensors_c bump_sensors;
PID_c left_pid;
PID_c right_pid;

// -------------------- BUTTON --------------------
const int BUTTON_A_PIN = 14;

// -------------------- TIMING --------------------
uint32_t lastLoop = 0;
const uint32_t LOOP_TIME_MS = 20;

// speed estimation timing
unsigned long speed_est_ts = 0;
const uint32_t SPEED_EST_MS = 10;
const float ALPHA = 0.2f;

// -------------------- SPEED ESTIMATION --------------------
long last_e0 = 0;
long last_e1 = 0;

float right_speed = 0.0f;
float left_speed = 0.0f;
float last_speed_e0 = 0.0f;
float last_speed_e1 = 0.0f;

// -------------------- BUMP SIGNALS --------------------
float L = 0.0f;
float R = 0.0f;
float total = 0.0f;

// baseline raw discharge times (beacon OFF)
float bump_baseline[BUMP_NUM_SENSORS];

// -------------------- TARGET SAMPLE --------------------
float targetTotal = 0.0f;
float targetDiff = 0.0f;

float lowerThreshold = 0.0f;
float upperThreshold = 0.0f;

float targetSampleSumTotal = 0.0f;
float targetSampleSumDiff = 0.0f;
int targetSampleCount = 0;

float minTotalSeen = 100000.0f;
float maxTotalSeen = -100000.0f;

// -------------------- TUNING --------------------
// target capture time
const uint32_t TARGET_SAMPLE_TIME_MS = 2000;

// beacon detection thresholds
// You said background is around 10 and beacon-on around 100,
// so these are good starting values.
const float BEACON_OFF_TOTAL = 20.0f;
const float BEACON_ON_TOTAL  = 40.0f;
const uint32_t LOST_TIMEOUT_MS = 200;

// require several consecutive samples to change state
const int REQUIRED_ON_COUNT = 3;
const int REQUIRED_OFF_COUNT = 3;

// drive control
const float MAX_FWD_SPEED_DEMAND = 0.45f;
const float SPEED_GAIN = 0.015f;
const float TARGET_DEADBAND = 2.0f;

// threshold band built from observed target variation
const float BAND_MARGIN_FRACTION = 0.20f;
const float MIN_BAND_MARGIN = 4.0f;

// pwm limits / floor
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR_FWD = 18;
const int PWM_FLOOR_TURN = 20;

// pid gains
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

// flip these if signs are wrong on your robot
const float FOLLOW_SIGN = 1.0f;
const float TURN_SIGN = -1.0f;

// offset / turning control
const float TURN_SPEED = 0.16f;             // fixed turn speed in ALIGNING mode
const float TURN_TRIM_GAIN = 0.006f;        // small turn trim in DRIVING mode
const float MAX_TURN_SPEED_DEMAND = 0.10f;

const float DIFF_ENTER = 8.0f;              // enter ALIGNING if abs(diffError) exceeds this
const float DIFF_EXIT  = 3.0f;              // return to DRIVING when abs(diffError) below this
const float DIFF_BIG   = 10.0f;             // reduce forward motion if diff error large

const float BASE_CHASE_SPEED = 0.18f;      // minimum forward speed when too far
const float FORWARD_PRIORITY_ERR = 6.0f;   // if total error exceeds this, prioritise forward
const float MAX_TURN_RATIO = 0.35f;        // turn demand no more than 35% of forward dem


// -------------------- STATE --------------------
enum RobotState {
  WAITING_FOR_TARGET_BUTTON,
  SAMPLING_TARGET,
  WAITING_FOR_BEACON_OFF,
  WAITING_FOR_BEACON_ON,
  FOLLOWING
};

enum TrackMode {
  SEARCHING,
  ALIGNING,
  DRIVING
};

RobotState robotState = WAITING_FOR_TARGET_BUTTON;
TrackMode trackMode = SEARCHING;

bool lastButtonState = HIGH;
uint32_t targetSampleStart = 0;
uint32_t beaconLostStart = 0;
float smoothedDrive = 0.0f;

// latched turn direction: +1 or -1
int turnDir = 1;

// counters for hysteresis transitions
int onCount = 0;
int offCount = 0;

// -------------------- OLED --------------------
const char* getStateName(RobotState s)
{
  switch (s) {
    case WAITING_FOR_TARGET_BUTTON: return "WAIT_TGT";
    case SAMPLING_TARGET:           return "SAMPLE";
    case WAITING_FOR_BEACON_OFF:    return "WAIT_OFF";
    case WAITING_FOR_BEACON_ON:     return "WAIT_ON";
    case FOLLOWING:                 return "FOLLOW";
    default:                        return "UNKNOWN";
  }
}

const char* getTrackName(TrackMode m)
{
  switch (m) {
    case SEARCHING: return "SEARCH";
    case ALIGNING:  return "ALIGN";
    case DRIVING:   return "DRIVE";
    default:        return "UNK";
  }
}

void drawFollowerOLED()
{
  display.clear();

  display.gotoXY(0, 0);
  display.print(getStateName(robotState));

  display.gotoXY(0, 1);
  display.print(total, 1);
  display.print("/");
  display.print(targetTotal, 1);
}

// -------------------- HELPERS --------------------
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int applyPwmFloor(int pwm, float demand_speed, int floorVal) {
  if (fabsf(demand_speed) < 1e-6f) return 0;
  if (abs(pwm) < floorVal) {
    pwm = (demand_speed > 0.0f) ? floorVal : -floorVal;
  }
  return pwm;
}

void stopRobot() {
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

void speedcalc(unsigned long elapsed_time) {
  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;

  last_e0 = count_e0;
  last_e1 = count_e1;

  float speed_e0 = (float)delta_e0 / (float)elapsed_time;
  float speed_e1 = (float)delta_e1 / (float)elapsed_time;

  // If one wheel sign is wrong, flip one of these:
  // speed_e1 = -speed_e1;

  right_speed = ALPHA * speed_e0 + (1.0f - ALPHA) * last_speed_e0;
  left_speed  = ALPHA * speed_e1 + (1.0f - ALPHA) * last_speed_e1;

  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
}

void updateWheelSpeedsIfNeeded() {
  unsigned long elapsed_time = millis() - speed_est_ts;
  if (elapsed_time >= SPEED_EST_MS) {
    speedcalc(elapsed_time);
    speed_est_ts = millis();
  }
}

// More IR -> smaller raw reading -> bigger signal
float readingToSignal(float reading, float baseline) {
  if (baseline <= 1.0f) return 0.0f;

  float x = 100.0f * (baseline - reading) / baseline;

  if (x < 0.0f) x = 0.0f;
  if (x > 100.0f) x = 100.0f;

  return x;
}

float median3(float a, float b, float c)
{
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    float t = a;
    a = b;
    b = t;
  }
  return b;
}

void updateBumpSignals() {
  float L1, L2, L3, R1, R2, R3;

  bump_sensors.readSensorsDigital();
  L1 = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
  R1 = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

  bump_sensors.readSensorsDigital();
  L2 = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
  R2 = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

  bump_sensors.readSensorsDigital();
  L3 = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
  R3 = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

  L = median3(L1, L2, L3);
  R = median3(R1, R2, R3);
  total = L + R;
}

void measureBumpBaseline() {
  const int N = 80;

  for (int s = 0; s < BUMP_NUM_SENSORS; s++) {
    bump_baseline[s] = 0.0f;
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

  Serial.print("Baseline L raw = ");
  Serial.print(bump_baseline[0]);
  Serial.print("  R raw = ");
  Serial.println(bump_baseline[1]);
}

void startTargetSampling() {
  targetSampleStart = millis();
  targetSampleSumTotal = 0.0f;
  targetSampleSumDiff = 0.0f;
  targetSampleCount = 0;
  minTotalSeen = 100000.0f;
  maxTotalSeen = -100000.0f;
}

void finishTargetSampling() {
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = targetSampleSumTotal / (float)targetSampleCount;
  targetDiff  = targetSampleSumDiff  / (float)targetSampleCount;

  float observedSpread = maxTotalSeen - minTotalSeen;
  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  Serial.println("target sampling done");
  Serial.print("targetTotal,"); Serial.println(targetTotal);
  Serial.print("targetDiff,");  Serial.println(targetDiff);
  Serial.print("lowerThreshold,"); Serial.println(lowerThreshold);
  Serial.print("upperThreshold,"); Serial.println(upperThreshold);
}

void driveAtDemands(float leftDemand, float rightDemand) {
  leftDemand  = clampFloat(leftDemand,  -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);
  rightDemand = clampFloat(rightDemand, -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);

  leftDemand  *= FOLLOW_SIGN;
  rightDemand *= FOLLOW_SIGN;

  float l_pwm_f = left_pid.update(leftDemand, left_speed);
  float r_pwm_f = right_pid.update(rightDemand, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  int floorL = (leftDemand == rightDemand) ? PWM_FLOOR_FWD : PWM_FLOOR_TURN;
  int floorR = (leftDemand == rightDemand) ? PWM_FLOOR_FWD : PWM_FLOOR_TURN;

  l_pwm = applyPwmFloor(l_pwm, leftDemand, floorL);
  r_pwm = applyPwmFloor(r_pwm, rightDemand, floorR);

  motors.setPWM(l_pwm, r_pwm);
}

void driveAtDemand(float demandSpeed) {
  driveAtDemands(demandSpeed, demandSpeed);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  motors.initialise();
  bump_sensors.initialiseForDigital();
  setupEncoder0();
  setupEncoder1();

  bump_sensors.timeout_us = 2500;
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  speed_est_ts = millis();

  Serial.println("Measure baseline: keep leader beacon OFF");
  measureBumpBaseline();

  Serial.println("Place leader at desired offset, beacon ON, then press follower button A");

  robotState = WAITING_FOR_TARGET_BUTTON;
  trackMode = SEARCHING;
  drawFollowerOLED();
  oled_ts = millis();
  lastLoop = millis();
}

// -------------------- LOOP --------------------
void loop() {
  updateWheelSpeedsIfNeeded();

  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  updateBumpSignals();

  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (robotState == WAITING_FOR_TARGET_BUTTON) {
    stopRobot();

    if (lastButtonState == HIGH && buttonState == LOW) {
      delay(20);
      if (digitalRead(BUTTON_A_PIN) == LOW) {
        startTargetSampling();
        robotState = SAMPLING_TARGET;
      }
    }
  }
  else if (robotState == SAMPLING_TARGET) {
    stopRobot();

    if (total > BEACON_ON_TOTAL) {
      float diff = R - L;

      targetSampleSumTotal += total;
      targetSampleSumDiff += diff;
      targetSampleCount++;

      if (total < minTotalSeen) minTotalSeen = total;
      if (total > maxTotalSeen) maxTotalSeen = total;
    }

    if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
      finishTargetSampling();
      robotState = WAITING_FOR_BEACON_OFF;
      offCount = 0;
    }
  }
  else if (robotState == WAITING_FOR_BEACON_OFF) {
    stopRobot();

    if (total < BEACON_OFF_TOTAL) offCount++;
    else offCount = 0;

    if (offCount >= REQUIRED_OFF_COUNT) {
      offCount = 0;
      robotState = WAITING_FOR_BEACON_ON;
      onCount = 0;
    }
  }
  else if (robotState == WAITING_FOR_BEACON_ON) {
    stopRobot();

    if (total > BEACON_ON_TOTAL) onCount++;
    else onCount = 0;

    if (onCount >= REQUIRED_ON_COUNT) {
      onCount = 0;
      left_pid.reset();
      right_pid.reset();
      smoothedDrive = 0.0f;
      beaconLostStart = 0;
      trackMode = ALIGNING;
      robotState = FOLLOWING;
    }
  }
  else if (robotState == FOLLOWING) {
    if (total < BEACON_OFF_TOTAL) {
      if (beaconLostStart == 0) beaconLostStart = millis();

      if (millis() - beaconLostStart > LOST_TIMEOUT_MS) {
        stopRobot();
        robotState = WAITING_FOR_BEACON_ON;
        onCount = 0;
        trackMode = SEARCHING;
      } else {
        // brief drop-out: keep turning in last chosen direction
        driveAtDemands(-turnDir * TURN_SPEED, turnDir * TURN_SPEED);
      }
    }
    else {
      beaconLostStart = 0;

      float diff = R - L;
      float diffError = targetDiff - diff;

      // ---------- ALIGNING ----------
      if (trackMode != ALIGNING && fabs(diffError) > DIFF_ENTER) {
        trackMode = ALIGNING;
        turnDir = (diffError > 0.0f) ? 1 : -1;
      }

      if (trackMode == ALIGNING) {
        if (fabs(diffError) < DIFF_EXIT) {
          trackMode = DRIVING;
          stopRobot();
        } else {
          driveAtDemands(-turnDir * TURN_SPEED, turnDir * TURN_SPEED);
        }
      }
      else { // ---------- DRIVING ----------
        float totalError = (targetTotal - TARGET_DEADBAND) - total;
        float desiredDrive = 0.0f;

        // If too far, drive forward.
        // Add a base chase speed so it starts moving promptly.
        if (totalError > 0.0f) {
          desiredDrive = BASE_CHASE_SPEED + SPEED_GAIN * totalError;
        } else {
          desiredDrive = 0.0f;
        }

        desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);

        float desiredTurn = 0.0f;

        // If we are significantly too far away, prioritise forward motion
        // and suppress steering trim completely.
        if (totalError <= FORWARD_PRIORITY_ERR) {
          // only apply trim when not badly behind
          if (fabs(diffError) < DIFF_EXIT) {
            diffError = 0.0f;
          }

          desiredTurn = TURN_SIGN * TURN_TRIM_GAIN * diffError;
          desiredTurn = clampFloat(desiredTurn, -MAX_TURN_SPEED_DEMAND, MAX_TURN_SPEED_DEMAND);

          // never let turn dominate forward motion
          float maxTurnAllowed = MAX_TURN_RATIO * desiredDrive;
          desiredTurn = clampFloat(desiredTurn, -maxTurnAllowed, maxTurnAllowed);
        } else {
          desiredTurn = 0.0f;
        }

        float leftDemand  = desiredDrive - desiredTurn;
        float rightDemand = desiredDrive + desiredTurn;

        smoothedDrive = desiredDrive;
        driveAtDemands(leftDemand, rightDemand);
      }
    }
  }

  lastButtonState = buttonState;

  Serial.print(total);
  Serial.print(",");
  Serial.print(targetTotal);
  Serial.print(",");
  Serial.print(targetDiff);
  Serial.print(",");
  Serial.print(L);
  Serial.print(",");
  Serial.print(R);
  Serial.print(",");
  Serial.print(R - L);
  Serial.print(",");
  Serial.print(smoothedDrive);
  Serial.print(",");
  Serial.print(left_speed);
  Serial.print(",");
  Serial.print(right_speed);
  Serial.print(",");
  Serial.print((int)robotState);
  Serial.print(",");
  Serial.println(getTrackName(trackMode));

  if (millis() - oled_ts >= OLED_UPDATE_MS) {
    oled_ts = millis();
    drawFollowerOLED();
  }
}
