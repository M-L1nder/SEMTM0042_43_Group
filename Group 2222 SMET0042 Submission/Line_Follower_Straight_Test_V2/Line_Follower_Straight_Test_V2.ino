#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "Kinematics.h"
#include <math.h>

// Straight-line reflective follower for 3Pi+ 32U4
//
// Procedure and Objective:
//  - Follower samples the leader's reflected IR signature while both robots
//    are stationary at the chosen X/Y offset
//  - Leader turns emitters OFF
//  - Leader then starts its straight reverse run with emitters ON
//  - Follower detects the returning signal, drives forward, and uses a small
//    steering trim to preserve the sampled lateral signature
//  - Once the follower has definitely chased and the received signal returns
//    to the original sampled strength band, it stops
//  - Robot logs odometry (x and y position) + sensor/error data into RAM and dumps CSV later
//    when button A is pressed after reconnecting USB (C for Leader robot)
//
// Button usage on follower:
//   1) Press A once to start target sampling
//   2) After the run, reconnect USB, open Serial Monitor at 115200,
//      press A once to dump CSV

Motors_c motors;
PID_c left_pid;
PID_c right_pid;
Kinematics_c pose;

// Pins
const int BUTTON_A_PIN = 14;

// Timing
const uint32_t LOOP_TIME_MS = 20;
const uint32_t SPEED_EST_MS = 10;
const uint32_t POSE_MS = 10;
const uint32_t TARGET_SAMPLE_TIME_MS = 2000;
const uint32_t TARGET_SETTLE_MS = 250;
const uint32_t LOST_TIMEOUT_MS = 350;
const uint32_t STOP_CONFIRM_MS = 250;
const uint32_t NO_TURN_AFTER_START_MS = 250;
const float ALPHA = 0.2f;

// Sensing
const int BASELINE_SAMPLES = 80;
const float BAND_MARGIN_FRACTION = 0.10f;
const float MIN_BAND_MARGIN = 0.8f;

// Drive tuning
const float BASE_CHASE_SPEED = 0.35f;
const float SPEED_GAIN = 0.010f;
const float MAX_FWD_SPEED_DEMAND = 0.40f;
const float TARGET_DEADBAND = 1.5f;

// IR trim (weak secondary correction)
const float TURN_TRIM_GAIN = 0.01f;
const float MAX_TURN_SPEED_DEMAND = 0.02f;
const float DIFF_DEADBAND = 0.10f;
const float MAX_TURN_RATIO = 0.12f;
const float FORWARD_PRIORITY_ERR = 4.0f;

const float FOLLOW_SIGN = 1.0f;
const float TURN_SIGN = -1.0f;

// Primary straight-line stabiliser: heading hold
const float HEADING_HOLD_GAIN = 0.12f;
const float MAX_HEADING_TURN = 0.05f;
const float HEADING_DEADBAND = 0.03f;

// IR lateral trim (secondary, very weak)
const float IR_TRIM_GAIN = 0.01f;
const float MAX_IR_TRIM = 0.02f;

// Leader presence thresholds are derived from targetTotal after sampling
float leaderOnThreshold  = 0.0f;
float leaderOffThreshold = 0.0f;

// PWM/PID
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR_FWD = 18;
const int PWM_FLOOR_TURN = 20;
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

// Starting offsets of runs (not needed actually as I'm physically changing anyway)
const float START_X_MM = 100.0f; // set to the actual starting X offset for this run
const float START_Y_MM = 0.0f;   // set to the actual starting Y offset for this run

// FSM
enum RobotState {
  WAITING_FOR_TARGET_BUTTON,
  SAMPLING_TARGET,
  WAITING_FOR_LEADER_OFF,
  WAITING_FOR_LEADER_ON,
  FOLLOWING,
  STOPPED
};

RobotState robotState = WAITING_FOR_TARGET_BUTTON;


// Timing constants/states
uint32_t lastLoop = 0;
uint32_t speed_est_ts = 0;
uint32_t pose_ts = 0;
uint32_t targetSampleStart = 0;
uint32_t leaderLostStart = 0;
uint32_t inBandStart = 0;
uint32_t followMotionStartTime = 0;
uint32_t waitOffEnteredAt = 0;
uint32_t waitOnEnteredAt = 0;

// Wheel speed
long last_e0 = 0;
long last_e1 = 0;
float right_speed = 0.0f;
float left_speed = 0.0f;
float last_speed_e0 = 0.0f;
float last_speed_e1 = 0.0f;

// Line sensor data
float raw[NUM_SENSORS];
float baseline[NUM_SENSORS];
float sig[NUM_SENSORS];

float totalSignal = 0.0f;
float leftAgg = 0.0f;
float rightAgg = 0.0f;
float diffLR = 0.0f;
float filteredTotal = 0.0f;
float normDiff = 0.0f;
float filteredNormDiff = 0.0f;

const float LEFT_MASK[NUM_SENSORS] = {1.0f, 0.8f, 0.2f, 0.0f, 0.0f};
const float RIGHT_MASK[NUM_SENSORS] = {0.0f, 0.0f, 0.2f, 0.8f, 1.0f};

// Target sig
float targetTotal = 0.0f;
float targetDiff = 0.0f;
float targetNormDiff = 0.0f;
float lowerThreshold = 0.0f;
float upperThreshold = 0.0f;
float tgtSumTotal = 0.0f;
float tgtSumDiff = 0.0f;
int   targetSampleCount = 0;
float minTotalSeen = 100000.0f;
float maxTotalSeen = -100000.0f;

// Control state
bool lastButtonState = HIGH;
float smoothedDrive = 0.0f;
bool hasStartedFollowingMotion = false;
bool hasBeenOutsideTargetBand = false;

// heading hold state
float targetHeading = 0.0f;

// csv logging
const uint32_t LOG_MS = 50;
const int LOG_SIZE = 95;

struct LogEntry {
  int32_t x_centi;
  int32_t y_centi;
  uint32_t t_ms;
};

LogEntry logBuffer[LOG_SIZE];
int logIndex = 0;
uint32_t lastLogTime = 0;
bool loggingActive = false;
bool logReadyToDump = false;

// Helper functions
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float wrapAngle(float a) {
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

int applyPwmFloor(int pwm, float demand, int floorVal) {
  if (fabsf(demand) < 1e-6f) return 0;
  if (abs(pwm) < floorVal) pwm = (demand > 0.0f) ? floorVal : -floorVal;
  return pwm;
}

void ensureEmittersOff() {
  pinMode(EMIT_PIN, INPUT);
}

void stopRobot() {
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
  smoothedDrive = 0.0f;
}

// Kinematics
void speedcalc(unsigned long elapsed) {
  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;
  last_e0 = count_e0;
  last_e1 = count_e1;

  float spd_e0 = (float)delta_e0 / (float)elapsed;
  float spd_e1 = (float)delta_e1 / (float)elapsed;

  right_speed = ALPHA * spd_e0 + (1.0f - ALPHA) * last_speed_e0;
  left_speed  = ALPHA * spd_e1 + (1.0f - ALPHA) * last_speed_e1;
  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
}

void updateWheelSpeedsIfNeeded() {
  unsigned long elapsed = millis() - speed_est_ts;
  if (elapsed >= SPEED_EST_MS) {
    speedcalc(elapsed);
    speed_est_ts = millis();
  }
}

void updatePoseIfNeeded() {
  uint32_t now = millis();
  if (now - pose_ts >= POSE_MS) {
    pose_ts = now;
    pose.update();
  }
}

// Sensor-related functions
void readRawSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    raw[i] = (float)analogRead(sensor_pins[i]);
  }
}

void computeSignals() {
  totalSignal = 0.0f;
  leftAgg = 0.0f;
  rightAgg = 0.0f;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (baseline[i] < 1.0f) {
      sig[i] = 0.0f;
    } else {
      float s = 100.0f * (baseline[i] - raw[i]) / baseline[i];
      if (s < 0.0f) s = 0.0f;
      if (s > 100.0f) s = 100.0f;
      sig[i] = s;
    }
    totalSignal += sig[i];
    leftAgg += LEFT_MASK[i] * sig[i];
    rightAgg += RIGHT_MASK[i] * sig[i];
  }

  diffLR = rightAgg - leftAgg;
  if (totalSignal > 1.0f) {
    normDiff = diffLR / totalSignal;
  } else {
    normDiff = 0.0f;
  }
  
  filteredNormDiff = 0.7f * filteredNormDiff + 0.3f * normDiff;
  filteredTotal = 0.5f * filteredTotal + 0.5f * totalSignal;
}

void updateLineSensors() {
  readRawSensors();
  computeSignals();
}

void measureBaseline() {
  Serial.println(F("# Measuring ambient baseline with leader emitters OFF"));

  for (int i = 0; i < NUM_SENSORS; i++) baseline[i] = 0.0f;

  for (int n = 0; n < BASELINE_SAMPLES; n++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      baseline[i] += (float)analogRead(sensor_pins[i]);
    }
    delay(5);
  }

  for (int i = 0; i < NUM_SENSORS; i++) baseline[i] /= (float)BASELINE_SAMPLES;
  filteredTotal = 0.0f;

  Serial.print(F("# Baseline DN1..DN5: "));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(baseline[i], 1);
    if (i < NUM_SENSORS - 1) Serial.print(F(", "));
  }
  Serial.println();
}

// Target sampling
void startTargetSampling() {
  targetSampleStart = millis();
  tgtSumTotal = 0.0f;
  tgtSumDiff = 0.0f;
  targetSampleCount = 0;
  minTotalSeen = 100000.0f;
  maxTotalSeen = -100000.0f;
  filteredTotal = 0.0f;
  filteredNormDiff = 0.0f;
  Serial.println(F("# Sampling target signature"));
}

void accumulateTargetSample() {
  if ((millis() - targetSampleStart) <= TARGET_SETTLE_MS) return;

  tgtSumTotal += totalSignal;
  tgtSumDiff += diffLR;
  targetSampleCount++;

  if (totalSignal < minTotalSeen) minTotalSeen = totalSignal;
  if (totalSignal > maxTotalSeen) maxTotalSeen = totalSignal;
}

void finishTargetSampling() {
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = tgtSumTotal / (float)targetSampleCount;
  targetDiff  = tgtSumDiff / (float)targetSampleCount;

  if (targetTotal > 1.0f) {
    targetNormDiff = targetDiff / targetTotal;
  } else {
    targetNormDiff = 0.0f;
  }
 
  float spread = maxTotalSeen - minTotalSeen;
  float bandMargin = spread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  leaderOnThreshold  = 0.25f * targetTotal;
  leaderOffThreshold = 0.12f * targetTotal;

  Serial.println(F("# Target sampling complete"));
  Serial.print(F("# targetTotal,")); Serial.println(targetTotal, 2);
  Serial.print(F("# targetDiff,")); Serial.println(targetDiff, 2);
  Serial.print(F("# lowerThreshold,")); Serial.println(lowerThreshold, 2);
  Serial.print(F("# upperThreshold,")); Serial.println(upperThreshold, 2);
  Serial.print(F("# leaderOnThreshold,")); Serial.println(leaderOnThreshold, 2);
  Serial.print(F("# leaderOffThreshold,")); Serial.println(leaderOffThreshold, 2);
}

// Logging
void startLogging() {
  logIndex = 0;
  loggingActive = true;
  logReadyToDump = false;
  lastLogTime = millis() - LOG_MS;
}

void finishLogging() {
  loggingActive = false;
  logReadyToDump = true;
  Serial.print(F("# finishLogging called. logIndex="));
  Serial.println(logIndex);
}
void logData() {
  if (!loggingActive) return;

  uint32_t now = millis();
  if (now - lastLogTime < LOG_MS) return;
  lastLogTime = now;

  if (logIndex >= LOG_SIZE) return;

  logBuffer[logIndex].x_centi = (int32_t)(pose.x * 100.0f);
  logBuffer[logIndex].y_centi = (int32_t)(pose.y * 100.0f);
  logBuffer[logIndex].t_ms = now;
  logIndex++;
}

void dumpLogCSV() {
  Serial.println(F("i,x_mm,y_mm,ts"));
  for (int i = 0; i < logIndex; i++) {
    Serial.print(i); Serial.print(",");
    Serial.print(logBuffer[i].x_centi / 100.0f, 2); Serial.print(",");
    Serial.print(logBuffer[i].y_centi / 100.0f, 2); Serial.print(",");
    Serial.println(logBuffer[i].t_ms);
  }
}

// Drive
void driveAtDemands(float leftDemand, float rightDemand) {
  leftDemand  = clampFloat(leftDemand,  -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);
  rightDemand = clampFloat(rightDemand, -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);

  leftDemand  *= FOLLOW_SIGN;
  rightDemand *= FOLLOW_SIGN;

  float l_pwm_f = left_pid.update(leftDemand, left_speed);
  float r_pwm_f = right_pid.update(rightDemand, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  bool turning = (fabsf(leftDemand - rightDemand) > 0.01f);
  int floorL = turning ? PWM_FLOOR_TURN : PWM_FLOOR_FWD;
  int floorR = turning ? PWM_FLOOR_TURN : PWM_FLOOR_FWD;

  l_pwm = applyPwmFloor(l_pwm, leftDemand, floorL);
  r_pwm = applyPwmFloor(r_pwm, rightDemand, floorR);

  motors.setPWM(l_pwm, r_pwm);
}

// Setup
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("### FOLLOWER BOOTED ###"));
  ensureEmittersOff();

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensor_pins[i], INPUT_PULLUP);
  }

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();

  pose.initialise(0.0f, 0.0f, 0.0f);

  last_e0 = count_e0;
  last_e1 = count_e1;
  speed_est_ts = millis();
  pose_ts = millis();
  lastLoop = millis();

  measureBaseline();
  Serial.println(F("# Ready: place robots, turn leader beacon ON with leader A, then press follower A once"));
}

// Main loop
void loop() {
  updateWheelSpeedsIfNeeded();
  updatePoseIfNeeded();

  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  updateLineSensors();

  bool buttonState = digitalRead(BUTTON_A_PIN);
  float totalError = 0.0f;
  float diffError = 0.0f;
  float cmdDrive = 0.0f;
  float cmdTurn = 0.0f;

  // dump log if available
  if (lastButtonState == HIGH && buttonState == LOW) {
  delay(20);
  if (digitalRead(BUTTON_A_PIN) == LOW) {
    Serial.print(F("# A pressed. logReadyToDump="));
    Serial.print(logReadyToDump);
    Serial.print(F(" logIndex="));
    Serial.print(logIndex);
    Serial.print(F(" state="));
    Serial.println((int)robotState);

    if (logReadyToDump) {
      dumpLogCSV();
      logReadyToDump = false;
      lastButtonState = buttonState;
      return;
    }
    if (robotState == WAITING_FOR_TARGET_BUTTON) {
      startTargetSampling();
      robotState = SAMPLING_TARGET;
    }
  }
}

  switch (robotState) {
    case WAITING_FOR_TARGET_BUTTON:
      stopRobot();
      break;

    case SAMPLING_TARGET:
      stopRobot();
      accumulateTargetSample();
      if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
        finishTargetSampling();
        waitOffEnteredAt = millis();
        robotState = WAITING_FOR_LEADER_OFF;
      }
      break;

    case WAITING_FOR_LEADER_OFF:
      stopRobot();
      if (millis() - waitOffEnteredAt > 150) {
        if (filteredTotal < leaderOffThreshold) {
          waitOnEnteredAt = millis();
          robotState = WAITING_FOR_LEADER_ON;
          Serial.println(F("# Leader OFF detected. Press leader B to start run."));
        }
      }
      break;

    case WAITING_FOR_LEADER_ON:
      stopRobot();
      if (millis() - waitOnEnteredAt > 150) {
        if (filteredTotal > leaderOnThreshold) {
          left_pid.reset();
          right_pid.reset();
          smoothedDrive = 0.0f;
          hasStartedFollowingMotion = false;
          hasBeenOutsideTargetBand = false;
          followMotionStartTime = 0;
          leaderLostStart = 0;
          inBandStart = 0;
          filteredNormDiff = normDiff;
          targetHeading = pose.theta;   // hold the initial heading from the start pose
          startLogging();
          robotState = FOLLOWING;
          Serial.println(F("# FOLLOWING started"));
        }
      }
      break;  

    case FOLLOWING: {
      if (filteredTotal < leaderOffThreshold) {
        if (leaderLostStart == 0) leaderLostStart = millis();
        if (millis() - leaderLostStart > LOST_TIMEOUT_MS) {
          stopRobot();
          finishLogging();
          robotState = STOPPED;
          Serial.println(F("# STOPPED: signal lost"));
          break;
        }
      } else {
        leaderLostStart = 0;
      }

      totalError = targetTotal - totalSignal;
      diffError  = targetNormDiff - filteredNormDiff;

      float activeTotalError = (fabsf(totalError) > TARGET_DEADBAND) ? (totalError - TARGET_DEADBAND) : 0.0f;
      if (activeTotalError > 0.0f) {
        cmdDrive = BASE_CHASE_SPEED + SPEED_GAIN * activeTotalError;
      } else {
        cmdDrive = 0.0f;
      }
      cmdDrive = clampFloat(cmdDrive, 0.0f, MAX_FWD_SPEED_DEMAND);
      smoothedDrive = 0.5f * cmdDrive + 0.5f * smoothedDrive;
      
      if (smoothedDrive > 0.03f) {
        if (!hasStartedFollowingMotion) {
          hasStartedFollowingMotion = true;
          followMotionStartTime = millis();
        }
      }
      
      bool steeringEnabled = true;
      if (hasStartedFollowingMotion && (millis() - followMotionStartTime < NO_TURN_AFTER_START_MS)) {
        steeringEnabled = false;
      }
      
      if (!steeringEnabled) {
        cmdTurn = 0.0f;
      } else {
        // Primary term: hold heading straight
        float headingError = wrapAngle(targetHeading - pose.theta);
        float headingTurn = 0.0f;
      
        if (fabsf(headingError) > HEADING_DEADBAND) {
          headingTurn = HEADING_HOLD_GAIN * headingError;
          headingTurn = clampFloat(headingTurn, -MAX_HEADING_TURN, MAX_HEADING_TURN);
        }
      
        // Secondary term: very weak IR trim
        float irTurn = 0.0f;
        if (fabsf(diffError) > DIFF_DEADBAND && activeTotalError <= FORWARD_PRIORITY_ERR) {
          irTurn = TURN_SIGN * IR_TRIM_GAIN * diffError;
          irTurn = clampFloat(irTurn, -MAX_IR_TRIM, MAX_IR_TRIM);
        }
      
        cmdTurn = headingTurn + irTurn;
      
        float maxTurnAllowed = MAX_TURN_RATIO * smoothedDrive;
        cmdTurn = clampFloat(cmdTurn, -maxTurnAllowed, maxTurnAllowed);
      }

      float leftDemand  = smoothedDrive - cmdTurn;
      float rightDemand = smoothedDrive + cmdTurn;
      driveAtDemands(leftDemand, rightDemand);

      if (totalSignal < lowerThreshold) {
        hasBeenOutsideTargetBand = true;
        inBandStart = 0;
      }

      if (hasStartedFollowingMotion && hasBeenOutsideTargetBand && totalSignal >= targetTotal) {
        if (inBandStart == 0) inBandStart = millis();
        if (millis() - inBandStart >= STOP_CONFIRM_MS) {
          stopRobot();
          finishLogging();
          robotState = STOPPED;
          Serial.println(F("# STOPPED: target strength recovered"));
        }
      } else {
        if (!(totalSignal >= targetTotal)) {
          inBandStart = 0;
        }
      }
      break;
    }

    case STOPPED:
      stopRobot();
      break;
  }

  logData();
  lastButtonState = buttonState;
}
