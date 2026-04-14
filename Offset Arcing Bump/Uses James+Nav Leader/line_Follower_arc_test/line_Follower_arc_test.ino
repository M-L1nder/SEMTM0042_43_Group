////#include "Motors.h"
////#include "LineSensors.h"
////#include "PID.h"
////#include "Encoders.h"
////#include "Kinematics.h"
////
////Motors_c motors;
////LineSensors_c line_sensors;
////PID_c left_pid;
////PID_c right_pid;
////Kinematics_c pose;
////
//////==========================================================================//
////// TIMING
//////==========================================================================//
////
////uint32_t lastLoop = 0;
////const uint32_t LOOP_TIME_MS = 10;
////
////unsigned long speed_est_ts = 0;
////unsigned long pose_ts = 0;
////const uint32_t SPEED_EST_MS = 10;
////const uint32_t POSE_MS = 10;
////
//////==========================================================================//
////// SPEED ESTIMATION
//////==========================================================================//
////
////const float ALPHA = 0.2f;
////
////long last_e0 = 0;
////long last_e1 = 0;
////
////float right_speed = 0.0f;
////float left_speed = 0.0f;
////float last_speed_e0 = 0.0f;
////float last_speed_e1 = 0.0f;
////
//////==========================================================================//
////// LINE SENSOR BASELINE / SIGNAL
//////==========================================================================//
////
////float line_baseline[NUM_SENSORS];
////float baselineTotal = 0.0f;
////float leaderThreshold = 0.0f;
////float leaderOffThreshold = 0.0f;
////
////float total = 0.0f;
////float filteredTotal = 0.0f;
////float prevTotal = 0.0f;
////float L = 0.0f;
////float C = 0.0f;
////float R = 0.0f;
////float smoothedLR = 0.0f;
////
//////==========================================================================//
////// TARGET SAMPLING
//////==========================================================================//
////
////const uint32_t TARGET_SAMPLE_TIME_MS = 2000;
////const uint32_t TARGET_SETTLE_MS = 300;
////
////uint32_t targetSampleStart = 0;
////float targetSampleSum = 0.0f;
////int targetSampleCount = 0;
////float minTotalSeen = 100000.0f;
////float maxTotalSeen = -100000.0f;
////
////float targetTotal = 0.0f;
////float targetLR = 0.0f;
////float lrSampleSum = 0.0f;
////float lowerThreshold = 0.0f;
////float upperThreshold = 0.0f;
////
////const float BAND_MARGIN_FRACTION = 0.20f;
////const float MIN_BAND_MARGIN = 15.0f;
////
//////==========================================================================//
////// MOVEMENT CONTROL
//////==========================================================================//
////
////float smoothedDrive = 0.0f;
////
////const float MAX_FWD_SPEED_DEMAND = 0.35f;
////const float SPEED_GAIN = 0.2f;
////const float TARGET_DEADBAND = 2.4f;
////
////const float STEER_GAIN = 0.0055f;
////const float MAX_STEER = 0.25f;
////
////const int PWM_MAX_ABS = 60;
////const int PWM_FLOOR_FWD = 18;
////
////const float DRIVE_KP = 15.0f;
////const float DRIVE_KI = 0.1f;
////const float DRIVE_KD = 0.0f;
////
////// stop / latch logic
////const uint32_t STOP_SETTLE_MS = 500;
////uint32_t belowDriveSince = 0;
////bool hasStartedFollowingMotion = false;
////const float FOLLOW_START_THRESHOLD = 0.03f;
////
////// no-turn startup window
////const uint32_t NO_TURN_AFTER_START_MS = 1000;
////uint32_t followMotionStartTime = 0;
////
////// leader lost timeout
////const uint32_t LOST_TIMEOUT_MS = 400;
////uint32_t leaderLostStart = 0;
////
//////==========================================================================//
////// FSM
//////==========================================================================//
////
////enum RobotState {
////  WAITING_FOR_TARGET_BUTTON,
////  SAMPLING_TARGET,
////  WAITING_FOR_LEADER_OFF,
////  WAITING_FOR_LEADER_ON,
////  FOLLOWING,
////  STOPPED
////};
////
////RobotState robotState = WAITING_FOR_TARGET_BUTTON;
////
//////==========================================================================//
////// LOGGING
//////==========================================================================//
////
////const uint32_t LOG_MS = 50;
////const int LOG_SIZE = 100;
////
////struct LogEntry {
////  uint32_t t_ms;
////  int16_t x_centi;
////  int16_t y_centi;
////};
////
////LogEntry logBuffer[LOG_SIZE];
////int logIndex = 0;
////uint32_t lastLogTime = 0;
////bool loggingActive = false;
////bool logReadyToDump = false;
////
////const int BUTTON_A_PIN = 14;
////bool lastButtonState = HIGH;
////
//////==========================================================================//
////// SERIAL PLOTTER
//////==========================================================================//
////
////const bool USE_SERIAL_PLOTTER = true;
////uint32_t plot_ts = 0;
////const uint32_t PLOT_MS = 50;
////
//////==========================================================================//
////// HELPERS
//////==========================================================================//
////
////float clampFloat(float x, float lo, float hi) {
////  if (x < lo) return lo;
////  if (x > hi) return hi;
////  return x;
////}
////
////int clampInt(int x, int lo, int hi) {
////  if (x < lo) return lo;
////  if (x > hi) return hi;
////  return x;
////}
////
////int applyPwmFloor(int pwm, float demand_speed, int floorVal) {
////  if (fabsf(demand_speed) < 1e-6f) return 0;
////  if (abs(pwm) < floorVal) {
////    pwm = (demand_speed > 0.0f) ? floorVal : -floorVal;
////  }
////  return pwm;
////}
////
////void stopRobot() {
////  motors.setPWM(0, 0);
////  left_pid.reset();
////  right_pid.reset();
////  smoothedDrive = 0.0f;
////
////  if (loggingActive) {
////    loggingActive = false;
////    logReadyToDump = true;
////  }
////}
////
////void logData() {
////  if (!loggingActive) return;
////
////  uint32_t now = millis();
////  if (now - lastLogTime < LOG_MS) return;
////  lastLogTime = now;
////
////  if (logIndex >= LOG_SIZE) return;
////
////  logBuffer[logIndex].x_centi = (int16_t)(pose.x * 100.0f);
////  logBuffer[logIndex].y_centi = (int16_t)((pose.y + 300.0f) * 100.0f);
////  logBuffer[logIndex].t_ms = now;
////  logIndex++;
////}
////
////const char* getStateName(RobotState s) {
////  switch (s) {
////    case WAITING_FOR_TARGET_BUTTON: return "WAIT_TGT";
////    case SAMPLING_TARGET:           return "SAMPLE";
////    case WAITING_FOR_LEADER_OFF:    return "WAIT_OFF";
////    case WAITING_FOR_LEADER_ON:     return "WAIT_ON";
////    case FOLLOWING:                 return "FOLLOW";
////    case STOPPED:                   return "STOPPED";
////    default:                        return "UNKNOWN";
////  }
////}
////
//////==========================================================================//
////// SPEED / POSE
//////==========================================================================//
////
////void speedcalc(unsigned long elapsed_time) {
////  long delta_e0 = count_e0 - last_e0;
////  long delta_e1 = count_e1 - last_e1;
////
////  last_e0 = count_e0;
////  last_e1 = count_e1;
////
////  float speed_e0 = float(delta_e0) / float(elapsed_time);
////  float speed_e1 = float(delta_e1) / float(elapsed_time);
////
////  right_speed = ALPHA * speed_e0 + (1.0f - ALPHA) * last_speed_e0;
////  left_speed  = ALPHA * speed_e1 + (1.0f - ALPHA) * last_speed_e1;
////
////  last_speed_e0 = right_speed;
////  last_speed_e1 = left_speed;
////}
////
////void updateWheelSpeedsIfNeeded() {
////  unsigned long elapsed_time = millis() - speed_est_ts;
////  if (elapsed_time >= SPEED_EST_MS) {
////    speedcalc(elapsed_time);
////    speed_est_ts = millis();
////  }
////}
////
////void updatePoseIfNeeded() {
////  unsigned long now = millis();
////  if (now - pose_ts >= POSE_MS) {
////    pose_ts = now;
////    pose.update();
////  }
////}
////
//////==========================================================================//
////// LINE SENSOR CONVERSION
//////==========================================================================//
////
////float readingToSignal(float reading, float baseline) {
////  if (baseline <= 1.0f) return 0.0f;
////
////  float x = 100.0f * (baseline - reading) / baseline;
////
////  if (x < 0.0f) x = 0.0f;
////  if (x > 100.0f) x = 100.0f;
////  return x;
////}
////
////void updateSensors() {
////  line_sensors.readSensorsDigital();
////
////  float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
////  float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
////  float s2 = readingToSignal(line_sensors.readings[2], line_baseline[2]);
////  float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
////  float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);
////
////  L = s0 + s1;
////  C = s2;
////  R = s3 + s4;
////
////  prevTotal = total;
////
////  // Using L + R only for distance-like total is usually more stable
////  total = L + R;
////
////  // Low-pass filter for threshold / loss decisions
////  filteredTotal = 0.7f * filteredTotal + 0.3f * total;
////}
////
////void measureLineBaseline() {
////  const int N = 80;
////
////  for (int i = 0; i < NUM_SENSORS; i++) {
////    line_baseline[i] = 0.0f;
////  }
////
////  for (int n = 0; n < N; n++) {
////    line_sensors.readSensorsDigital();
////    for (int i = 0; i < NUM_SENSORS; i++) {
////      line_baseline[i] += line_sensors.readings[i];
////    }
////    delay(5);
////  }
////
////  for (int i = 0; i < NUM_SENSORS; i++) {
////    line_baseline[i] /= (float)N;
////  }
////
////  baselineTotal = 0.0f;
////  for (int n = 0; n < 50; n++) {
////    line_sensors.readSensorsDigital();
////
////    float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
////    float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
////    float s2 = readingToSignal(line_sensors.readings[2], line_baseline[2]);
////    float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
////    float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);
////
////    // Match updateSensors(): L + R only
////    baselineTotal += (s0 + s1 + s3 + s4);
////    delay(5);
////  }
////
////  baselineTotal /= 50.0f;
////  filteredTotal = baselineTotal;
////}
////
//////==========================================================================//
////// TARGET SAMPLING
//////==========================================================================//
////
////void startTargetSampling() {
////  targetSampleStart = millis();
////  targetSampleSum = 0.0f;
////  targetSampleCount = 0;
////  lrSampleSum = 0.0f;
////  minTotalSeen = 100000.0f;
////  maxTotalSeen = -100000.0f;
////  total = 0.0f;
////  filteredTotal = baselineTotal;
////}
////
////void finishTargetSampling() {
////  if (targetSampleCount < 1) targetSampleCount = 1;
////
////  targetTotal = targetSampleSum / (float)targetSampleCount;
////  targetLR = lrSampleSum / (float)targetSampleCount;
////
////  leaderThreshold = baselineTotal + 0.4f * (targetTotal - baselineTotal);
////  leaderOffThreshold = baselineTotal + 0.2f * (targetTotal - baselineTotal);
////
////  float observedSpread = maxTotalSeen - minTotalSeen;
////  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
////  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;
////
////  lowerThreshold = targetTotal - bandMargin;
////  upperThreshold = targetTotal + bandMargin;
////
////  Serial.println(F("target sampling done"));
////  Serial.print(F("targetTotal,")); Serial.println(targetTotal);
////  Serial.print(F("lowerThreshold,")); Serial.println(lowerThreshold);
////  Serial.print(F("upperThreshold,")); Serial.println(upperThreshold);
////  Serial.print(F("leaderThreshold,")); Serial.println(leaderThreshold);
////  Serial.print(F("leaderOffThreshold,")); Serial.println(leaderOffThreshold);
////}
////
//////==========================================================================//
////// STATE FUNCTIONS
//////==========================================================================//
////
////void stateWaitingForButton() {
////  motors.setPWM(0, 0);
////}
////
////void stateSampling() {
////  motors.setPWM(0, 0);
////
////  if ((millis() - targetSampleStart) > TARGET_SETTLE_MS && filteredTotal > leaderThreshold) {
////    targetSampleSum += total;
////    lrSampleSum += (L - R);
////    targetSampleCount++;
////
////    if (total < minTotalSeen) minTotalSeen = total;
////    if (total > maxTotalSeen) maxTotalSeen = total;
////  }
////
////  if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
////    if (targetSampleCount < 20) {
////      robotState = WAITING_FOR_TARGET_BUTTON;
////      return;
////    }
////    finishTargetSampling();
////    robotState = WAITING_FOR_LEADER_OFF;
////  }
////}
////
////void stateWaitingForLeaderOff() {
////  motors.setPWM(0, 0);
////
////  if (filteredTotal < leaderOffThreshold) {
////    total = 0.0f;
////    filteredTotal = baselineTotal;
////    smoothedLR = 0.0f;
////    robotState = WAITING_FOR_LEADER_ON;
////  }
////}
////
////void stateWaitingForLeaderOn() {
////  motors.setPWM(0, 0);
////
////  if (filteredTotal > leaderThreshold) {
////    left_pid.reset();
////    right_pid.reset();
////    smoothedDrive = 0.0f;
////    smoothedLR = (L - R) - targetLR;
////    belowDriveSince = 0;
////    hasStartedFollowingMotion = false;
////    followMotionStartTime = 0;
////    leaderLostStart = 0;
////
////    logIndex = 0;
////    loggingActive = true;
////    logReadyToDump = false;
////    lastLogTime = millis() - LOG_MS;
////
////    robotState = FOLLOWING;
////  }
////}
////
//////void stateFollowing() {
//////  if (!loggingActive && !logReadyToDump) {
//////    logIndex = 0;
//////    loggingActive = true;
//////    lastLogTime = millis() - LOG_MS;
//////  }
//////
//////  // Gentler lost-target logic for line sensors
//////  if (filteredTotal < leaderOffThreshold) {
//////    if (leaderLostStart == 0) leaderLostStart = millis();
//////
//////    if (millis() - leaderLostStart > LOST_TIMEOUT_MS) {
//////      stopRobot();
//////      robotState = STOPPED;
//////      return;
//////    }
//////  } else {
//////    leaderLostStart = 0;
//////  }
//////
//////  float error = (targetTotal - TARGET_DEADBAND) - total;
//////  float desiredDrive = 0.0f;
//////
//////  if (total < targetTotal - TARGET_DEADBAND) {
//////    desiredDrive = SPEED_GAIN * error;
//////  }
//////
//////  desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);
//////  smoothedDrive = 0.3f * desiredDrive + 0.7f * smoothedDrive;
//////
//////  if (smoothedDrive > FOLLOW_START_THRESHOLD) {
//////    if (!hasStartedFollowingMotion) {
//////      hasStartedFollowingMotion = true;
//////      followMotionStartTime = millis();
//////    }
//////  }
//////
//////  if (smoothedDrive < 0.015f) {
//////    if (hasStartedFollowingMotion) {
//////      if (belowDriveSince == 0) belowDriveSince = millis();
//////
//////      motors.setPWM(0, 0);
//////
//////      if (millis() - belowDriveSince >= STOP_SETTLE_MS) {
//////        stopRobot();
//////        robotState = STOPPED;
//////      }
//////    } else {
//////      belowDriveSince = 0;
//////      motors.setPWM(0, 0);
//////    }
//////    return;
//////  } else {
//////    if (smoothedDrive > 0.12f) belowDriveSince = 0;
//////  }
//////
//////  float rawLRDiff = (L - R) - targetLR;
//////  bool steeringEnabled = true;
//////
//////  if (hasStartedFollowingMotion) {
//////    if (millis() - followMotionStartTime < NO_TURN_AFTER_START_MS) {
//////      steeringEnabled = false;
//////    }
//////  }
//////
//////  smoothedLR = 0.5f * rawLRDiff + 0.5f * smoothedLR;
//////
//////  float steer = 0.0f;
//////  if (steeringEnabled) {
//////    steer = clampFloat(-STEER_GAIN * smoothedLR, -MAX_STEER, MAX_STEER);
//////  }
//////
//////  float l_demand = smoothedDrive + steer;
//////  float r_demand = smoothedDrive - steer;
//////
//////  float l_pwm_f = left_pid.update(l_demand, left_speed);
//////  float r_pwm_f = right_pid.update(r_demand, right_speed);
//////
//////  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
//////  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
//////
//////  l_pwm = applyPwmFloor(l_pwm, l_demand, PWM_FLOOR_FWD);
//////  r_pwm = applyPwmFloor(r_pwm, r_demand, PWM_FLOOR_FWD);
//////
//////  motors.setPWM(l_pwm, r_pwm);
//////}
////
////void stateFollowing() {
////  if (!loggingActive && !logReadyToDump) {
////    logIndex = 0;
////    loggingActive = true;
////    lastLogTime = millis() - LOG_MS;
////  }
////
////  float error = (targetTotal - TARGET_DEADBAND) - total;
////  float desiredDrive = 0.0f;
////
////  if (total < targetTotal - TARGET_DEADBAND) {
////    desiredDrive = SPEED_GAIN * error;
////  }
////
////  desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);
////  smoothedDrive = 0.3f * desiredDrive + 0.7f * smoothedDrive;
////
////  float rawLRDiff = (L - R) - targetLR;
////  smoothedLR = 0.5f * rawLRDiff + 0.5f * smoothedLR;
////
////  float steer = 0.0f;
////  steer = clampFloat(-STEER_GAIN * smoothedLR, -MAX_STEER, MAX_STEER);
////
////  float l_demand = smoothedDrive + steer;
////  float r_demand = smoothedDrive - steer;
////
////  float l_pwm_f = left_pid.update(l_demand, left_speed);
////  float r_pwm_f = right_pid.update(r_demand, right_speed);
////
////  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
////  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
////
////  l_pwm = applyPwmFloor(l_pwm, l_demand, PWM_FLOOR_FWD);
////  r_pwm = applyPwmFloor(r_pwm, r_demand, PWM_FLOOR_FWD);
////
////  motors.setPWM(l_pwm, r_pwm);
////}
////
////void stateStopped() {
////  motors.setPWM(0, 0);
////}
////
//////==========================================================================//
////// SERIAL PLOTTER
//////==========================================================================//
////
////void plotDebugSignals() {
////  if (!USE_SERIAL_PLOTTER) return;
////
////  uint32_t now = millis();
////  if (now - plot_ts < PLOT_MS) return;
////  plot_ts = now;
////
////  float driveGate = (total < targetTotal - TARGET_DEADBAND) ? 100.0f : 0.0f;
////  float lostGate  = (filteredTotal < leaderOffThreshold) ? 100.0f : 0.0f;
////
////  Serial.print(total, 2);                    Serial.print(",");
////  Serial.print(filteredTotal, 2);            Serial.print(",");
////  Serial.print(targetTotal, 2);              Serial.print(",");
////  Serial.print(leaderThreshold, 2);          Serial.print(",");
////  Serial.print(leaderOffThreshold, 2);       Serial.print(",");
////  Serial.print(100 * smoothedDrive, 3);      Serial.print(",");
////  Serial.print(driveGate, 1);                Serial.print(",");
////  Serial.print(lostGate, 1);                 Serial.print(",");
////  Serial.println((int)robotState);
////}
////
//////==========================================================================//
////// SETUP
//////==========================================================================//
////
////void setup() {
////  Serial.begin(9600);
////  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
////
////  motors.initialise();
////  line_sensors.initialiseForDigital();
////  line_sensors.timeout_us = 9000;
////  setupEncoder0();
////  setupEncoder1();
////
////  pinMode(EMIT_PIN, INPUT);
////
////  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
////  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
////  left_pid.reset();
////  right_pid.reset();
////
////  pose.initialise(0.0f, 0.0f, 0.0f);
////
////  last_e0 = count_e0;
////  last_e1 = count_e1;
////  speed_est_ts = millis();
////  pose_ts = millis();
////
////  measureLineBaseline();
////
////  robotState = WAITING_FOR_TARGET_BUTTON;
////  smoothedDrive = 0.0f;
////  belowDriveSince = 0;
////  hasStartedFollowingMotion = false;
////  followMotionStartTime = 0;
////
////  lastLoop = millis();
////}
////
//////==========================================================================//
////// LOOP
//////==========================================================================//
////
////void loop() {
////  updateWheelSpeedsIfNeeded();
////  updatePoseIfNeeded();
////
////  if (millis() - lastLoop < LOOP_TIME_MS) return;
////  lastLoop += LOOP_TIME_MS;
////
////  updateSensors();
////
////  bool buttonState = digitalRead(BUTTON_A_PIN);
////
////  if (lastButtonState == HIGH && buttonState == LOW) {
////    delay(20);
////    if (digitalRead(BUTTON_A_PIN) == LOW) {
////      if (logReadyToDump) {
////        Serial.println(F("i,x_mm,y_mm,ts"));
////        for (int i = 0; i < logIndex; i++) {
////          float x = logBuffer[i].x_centi / 100.0f;
////          float y = logBuffer[i].y_centi / 100.0f - 300.0f;
////          Serial.print(i);
////          Serial.print(",");
////          Serial.print(x);
////          Serial.print(",");
////          Serial.print(y);
////          Serial.print(",");
////          Serial.println(logBuffer[i].t_ms);
////        }
////        logReadyToDump = false;
////        lastButtonState = buttonState;
////        return;
////      }
////
////      if (robotState == WAITING_FOR_TARGET_BUTTON) {
////        startTargetSampling();
////        robotState = SAMPLING_TARGET;
////      }
////    }
////  }
////
////  lastButtonState = buttonState;
////
////  switch (robotState) {
////    case WAITING_FOR_TARGET_BUTTON:
////      stateWaitingForButton();
////      break;
////    case SAMPLING_TARGET:
////      stateSampling();
////      break;
////    case WAITING_FOR_LEADER_OFF:
////      stateWaitingForLeaderOff();
////      break;
////    case WAITING_FOR_LEADER_ON:
////      stateWaitingForLeaderOn();
////      break;
////    case FOLLOWING:
////      stateFollowing();
////      break;
////    case STOPPED:
////      stateStopped();
////      break;
////  }
////
////  logData();
////  plotDebugSignals();
////}
//
//
//
//
//#include "Motors.h"
//#include "LineSensors.h"
//#include "PID.h"
//#include "Encoders.h"
//#include "Kinematics.h"
//
//Motors_c motors;
//LineSensors_c line_sensors;
//PID_c left_pid;
//PID_c right_pid;
//Kinematics_c pose;
//
////==========================================================================//
//// TIMING
////==========================================================================//
//
//uint32_t lastLoop = 0;
//const uint32_t LOOP_TIME_MS = 10;
//
//unsigned long speed_est_ts = 0;
//unsigned long pose_ts = 0;
//const uint32_t SPEED_EST_MS = 10;
//const uint32_t POSE_MS = 10;
//
////==========================================================================//
//// SPEED ESTIMATION
////==========================================================================//
//
//const float ALPHA = 0.2f;
//
//long last_e0 = 0;
//long last_e1 = 0;
//
//float right_speed = 0.0f;
//float left_speed = 0.0f;
//float last_speed_e0 = 0.0f;
//float last_speed_e1 = 0.0f;
//
////==========================================================================//
//// LINE SENSOR BASELINE / SIGNAL
////==========================================================================//
//
//float line_baseline[NUM_SENSORS];
//float baselineTotal = 0.0f;
//float leaderThreshold = 0.0f;
//float leaderOffThreshold = 0.0f;
//
//float total = 0.0f;
//float filteredTotal = 0.0f;
//float prevTotal = 0.0f;
//float L = 0.0f;
//float C = 0.0f;
//float R = 0.0f;
//float smoothedLR = 0.0f;
//
////==========================================================================//
//// TARGET SAMPLING
////==========================================================================//
//
//const uint32_t TARGET_SAMPLE_TIME_MS = 2000;
//const uint32_t TARGET_SETTLE_MS = 300;
//
//uint32_t targetSampleStart = 0;
//float targetSampleSum = 0.0f;
//int targetSampleCount = 0;
//float minTotalSeen = 100000.0f;
//float maxTotalSeen = -100000.0f;
//
//float targetTotal = 0.0f;
//float targetLR = 0.0f;
//float lrSampleSum = 0.0f;
//float lowerThreshold = 0.0f;
//float upperThreshold = 0.0f;
//
//const float BAND_MARGIN_FRACTION = 0.20f;
//const float MIN_BAND_MARGIN = 15.0f;
//
////==========================================================================//
//// MOVEMENT CONTROL
////==========================================================================//
//
//float smoothedDrive = 0.0f;
//
//const float MAX_FWD_SPEED_DEMAND = 0.35f;
//const float SPEED_GAIN = 0.2f;
//const float TARGET_DEADBAND = 2.4f;
//
//const float STEER_GAIN = 0.0055f;
//const float MAX_STEER = 0.25f;
//
//const int PWM_MAX_ABS = 60;
//const int PWM_FLOOR_FWD = 18;
//
//const float DRIVE_KP = 15.0f;
//const float DRIVE_KI = 0.1f;
//const float DRIVE_KD = 0.0f;
//
//// no-turn startup window
//const uint32_t NO_TURN_AFTER_START_MS = 1000;
//uint32_t followMotionStartTime = 0;
//bool hasStartedFollowingMotion = false;
//const float FOLLOW_START_THRESHOLD = 0.03f;
//// leader lost timeout
//const uint32_t LOST_TIMEOUT_MS = 500;
//uint32_t leaderLostStart = 0;
//
////==========================================================================//
//// FSM
////==========================================================================//
//
//enum RobotState {
//  WAITING_FOR_TARGET_BUTTON,
//  SAMPLING_TARGET,
//  WAITING_FOR_LEADER_OFF,
//  WAITING_FOR_LEADER_ON,
//  FOLLOWING,
//  STOPPED
//};
//
//RobotState robotState = WAITING_FOR_TARGET_BUTTON;
//
////==========================================================================//
//// LOGGING
////==========================================================================//
//
//const uint32_t LOG_MS = 50;
//const int LOG_SIZE = 100;
//
//struct LogEntry {
//  uint32_t t_ms;
//  int16_t x_centi;
//  int16_t y_centi;
//};
//
//LogEntry logBuffer[LOG_SIZE];
//int logIndex = 0;
//uint32_t lastLogTime = 0;
//bool loggingActive = false;
//bool logReadyToDump = false;
//
//const int BUTTON_A_PIN = 14;
//bool lastButtonState = HIGH;
//
////==========================================================================//
//// SERIAL PLOTTER
////==========================================================================//
//
//const bool USE_SERIAL_PLOTTER = true;
//uint32_t plot_ts = 0;
//const uint32_t PLOT_MS = 50;
//
////==========================================================================//
//// HELPERS
////==========================================================================//
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
//  smoothedDrive = 0.0f;
//
//  if (loggingActive) {
//    loggingActive = false;
//    logReadyToDump = true;
//  }
//}
//
//void logData() {
//  if (!loggingActive) return;
//
//  uint32_t now = millis();
//  if (now - lastLogTime < LOG_MS) return;
//  lastLogTime = now;
//
//  if (logIndex >= LOG_SIZE) return;
//
//  logBuffer[logIndex].x_centi = (int16_t)(pose.x * 100.0f);
//  logBuffer[logIndex].y_centi = (int16_t)((pose.y + 300.0f) * 100.0f);
//  logBuffer[logIndex].t_ms = now;
//  logIndex++;
//}
//
////==========================================================================//
//// SPEED / POSE
////==========================================================================//
//
//void speedcalc(unsigned long elapsed_time) {
//  long delta_e0 = count_e0 - last_e0;
//  long delta_e1 = count_e1 - last_e1;
//
//  last_e0 = count_e0;
//  last_e1 = count_e1;
//
//  float speed_e0 = float(delta_e0) / float(elapsed_time);
//  float speed_e1 = float(delta_e1) / float(elapsed_time);
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
//void updatePoseIfNeeded() {
//  unsigned long now = millis();
//  if (now - pose_ts >= POSE_MS) {
//    pose_ts = now;
//    pose.update();
//  }
//}
//
////==========================================================================//
//// LINE SENSOR CONVERSION
////==========================================================================//
//
//float readingToSignal(float reading, float baseline) {
//  if (baseline <= 1.0f) return 0.0f;
//
//  float x = 100.0f * (baseline - reading) / baseline;
//
//  if (x < 0.0f) x = 0.0f;
//  if (x > 100.0f) x = 100.0f;
//  return x;
//}
//
//void updateSensors() {
//  line_sensors.readSensorsDigital();
//
//  float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
//  float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
//  float s2 = readingToSignal(line_sensors.readings[2], line_baseline[2]);
//  float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
//  float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);
//
//  L = s0 + s1;
//  C = s2;
//  R = s3 + s4;
//
//  prevTotal = total;
//
//  // More stable than L+C+R for this use
//  total = L + R;
//
//  // Filter only for threshold / loss detection
//  filteredTotal = 0.7f * filteredTotal + 0.3f * total;
//}
//
//void measureLineBaseline() {
//  const int N = 80;
//
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    line_baseline[i] = 0.0f;
//  }
//
//  for (int n = 0; n < N; n++) {
//    line_sensors.readSensorsDigital();
//    for (int i = 0; i < NUM_SENSORS; i++) {
//      line_baseline[i] += line_sensors.readings[i];
//    }
//    delay(5);
//  }
//
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    line_baseline[i] /= (float)N;
//  }
//
//  baselineTotal = 0.0f;
//  for (int n = 0; n < 50; n++) {
//    line_sensors.readSensorsDigital();
//
//    float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
//    float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
//    float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
//    float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);
//
//    baselineTotal += (s0 + s1 + s3 + s4);
//    delay(5);
//  }
//
//  baselineTotal /= 50.0f;
//  filteredTotal = baselineTotal;
//}
//
////==========================================================================//
//// TARGET SAMPLING
////==========================================================================//
//
//void startTargetSampling() {
//  targetSampleStart = millis();
//  targetSampleSum = 0.0f;
//  targetSampleCount = 0;
//  lrSampleSum = 0.0f;
//  minTotalSeen = 100000.0f;
//  maxTotalSeen = -100000.0f;
//  total = 0.0f;
//  filteredTotal = baselineTotal;
//}
//
//void finishTargetSampling() {
//  if (targetSampleCount < 1) targetSampleCount = 1;
//
//  targetTotal = targetSampleSum / (float)targetSampleCount;
//  targetLR = lrSampleSum / (float)targetSampleCount;
//
//  leaderThreshold = baselineTotal + 0.4f * (targetTotal - baselineTotal);
//  leaderOffThreshold = baselineTotal + 0.2f * (targetTotal - baselineTotal);
//
//  float observedSpread = maxTotalSeen - minTotalSeen;
//  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
//  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;
//
//  lowerThreshold = targetTotal - bandMargin;
//  upperThreshold = targetTotal + bandMargin;
//
//  Serial.println(F("target sampling done"));
//  Serial.print(F("targetTotal,")); Serial.println(targetTotal);
//  Serial.print(F("lowerThreshold,")); Serial.println(lowerThreshold);
//  Serial.print(F("upperThreshold,")); Serial.println(upperThreshold);
//  Serial.print(F("leaderThreshold,")); Serial.println(leaderThreshold);
//  Serial.print(F("leaderOffThreshold,")); Serial.println(leaderOffThreshold);
//}
//
////==========================================================================//
//// STATE FUNCTIONS
////==========================================================================//
//
//void stateWaitingForButton() {
//  motors.setPWM(0, 0);
//}
//
//void stateSampling() {
//  motors.setPWM(0, 0);
//
//  if ((millis() - targetSampleStart) > TARGET_SETTLE_MS && filteredTotal > leaderThreshold) {
//    targetSampleSum += total;
//    lrSampleSum += (L - R);
//    targetSampleCount++;
//
//    if (total < minTotalSeen) minTotalSeen = total;
//    if (total > maxTotalSeen) maxTotalSeen = total;
//  }
//
//  if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
//    if (targetSampleCount < 20) {
//      robotState = WAITING_FOR_TARGET_BUTTON;
//      return;
//    }
//    finishTargetSampling();
//    robotState = WAITING_FOR_LEADER_OFF;
//  }
//}
//
//void stateWaitingForLeaderOff() {
//  motors.setPWM(0, 0);
//
//  if (filteredTotal < leaderOffThreshold) {
//    total = 0.0f;
//    filteredTotal = baselineTotal;
//    smoothedLR = 0.0f;
//    robotState = WAITING_FOR_LEADER_ON;
//  }
//}
//
//void stateWaitingForLeaderOn() {
//  motors.setPWM(0, 0);
//
//  if (filteredTotal > leaderThreshold) {
//    left_pid.reset();
//    right_pid.reset();
//    smoothedDrive = 0.0f;
//    smoothedLR = (L - R) - targetLR;
//    hasStartedFollowingMotion = false;
//    followMotionStartTime = 0;
//    leaderLostStart = 0;
//
//    logIndex = 0;
//    loggingActive = true;
//    logReadyToDump = false;
//    lastLogTime = millis() - LOG_MS;
//
//    robotState = FOLLOWING;
//  }
//}
//
//void stateFollowing() {
//  if (!loggingActive && !logReadyToDump) {
//    logIndex = 0;
//    loggingActive = true;
//    lastLogTime = millis() - LOG_MS;
//  }
//
//  // Desired forward drive from distance-like signal
//  float error = (targetTotal - TARGET_DEADBAND) - total;
//  float desiredDrive = 0.0f;
//
//  if (total < targetTotal - TARGET_DEADBAND) {
//    desiredDrive = SPEED_GAIN * error;
//  }
//
//  desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);
//
//  // Lost-leader handling:
//  // do NOT instantly zero drive on a brief dip,
//  // only stop after sustained loss.
//  if (filteredTotal < leaderOffThreshold) {
//    if (leaderLostStart == 0) leaderLostStart = millis();
//
//    if (millis() - leaderLostStart > LOST_TIMEOUT_MS) {
//      stopRobot();
//      robotState = STOPPED;
//      return;
//    }
//
//    // Hold recent drive briefly while checking if loss is real
//    desiredDrive = smoothedDrive;
//  } else {
//    leaderLostStart = 0;
//  }
//
//  smoothedDrive = 0.3f * desiredDrive + 0.7f * smoothedDrive;
//
//  if (smoothedDrive > FOLLOW_START_THRESHOLD) {
//    if (!hasStartedFollowingMotion) {
//      hasStartedFollowingMotion = true;
//      followMotionStartTime = millis();
//    }
//  }
//
//  float rawLRDiff = (L - R) - targetLR;
//  smoothedLR = 0.5f * rawLRDiff + 0.5f * smoothedLR;
//
//  bool steeringEnabled = true;
//  if (hasStartedFollowingMotion) {
//    if (millis() - followMotionStartTime < NO_TURN_AFTER_START_MS) {
//      steeringEnabled = false;
//    }
//  }
//
//  float steer = 0.0f;
//  if (steeringEnabled) {
//    steer = clampFloat(-STEER_GAIN * smoothedLR, -MAX_STEER, MAX_STEER);
//  }
//
//  float l_demand = smoothedDrive + steer;
//  float r_demand = smoothedDrive - steer;
//
//  float l_pwm_f = left_pid.update(l_demand, left_speed);
//  float r_pwm_f = right_pid.update(r_demand, right_speed);
//
//  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
//  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
//
//  l_pwm = applyPwmFloor(l_pwm, l_demand, PWM_FLOOR_FWD);
//  r_pwm = applyPwmFloor(r_pwm, r_demand, PWM_FLOOR_FWD);
//
//  motors.setPWM(l_pwm, r_pwm);
//}
//
//void stateStopped() {
//  motors.setPWM(0, 0);
//}
//
////==========================================================================//
//// SERIAL PLOTTER
////==========================================================================//
//
//void plotDebugSignals() {
//  if (!USE_SERIAL_PLOTTER) return;
//
//  uint32_t now = millis();
//  if (now - plot_ts < PLOT_MS) return;
//  plot_ts = now;
//
//  float driveGate = (total < targetTotal - TARGET_DEADBAND) ? 100.0f : 0.0f;
//  float lostGate  = (filteredTotal < leaderOffThreshold) ? 100.0f : 0.0f;
//
//  Serial.print(total, 2);               Serial.print(",");
//  Serial.print(filteredTotal, 2);       Serial.print(",");
//  Serial.print(targetTotal, 2);         Serial.print(",");
//  Serial.print(leaderThreshold, 2);     Serial.print(",");
//  Serial.print(leaderOffThreshold, 2);  Serial.print(",");
//  Serial.print(100 * smoothedDrive, 2); Serial.print(",");
//  Serial.print(driveGate, 1);           Serial.print(",");
//  Serial.print(lostGate, 1);            Serial.print(",");
//  Serial.println((int)robotState);
//}
//
////==========================================================================//
//// SETUP
////==========================================================================//
//
//void setup() {
//  Serial.begin(9600);
//  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
//
//  motors.initialise();
//  line_sensors.initialiseForDigital();
//  line_sensors.timeout_us = 8000;
//  setupEncoder0();
//  setupEncoder1();
//
//  pinMode(EMIT_PIN, INPUT);
//
//  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
//  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
//  left_pid.reset();
//  right_pid.reset();
//
//  pose.initialise(0.0f, 0.0f, 0.0f);
//
//  last_e0 = count_e0;
//  last_e1 = count_e1;
//  speed_est_ts = millis();
//  pose_ts = millis();
//
//  measureLineBaseline();
//
//  robotState = WAITING_FOR_TARGET_BUTTON;
//  smoothedDrive = 0.0f;
//  hasStartedFollowingMotion = false;
//  followMotionStartTime = 0;
//  leaderLostStart = 0;
//
//  lastLoop = millis();
//}
//
////==========================================================================//
//// LOOP
////==========================================================================//
//
//void loop() {
//  updateWheelSpeedsIfNeeded();
//  updatePoseIfNeeded();
//
//  if (millis() - lastLoop < LOOP_TIME_MS) return;
//  lastLoop += LOOP_TIME_MS;
//
//  updateSensors();
//
//  bool buttonState = digitalRead(BUTTON_A_PIN);
//
//  if (lastButtonState == HIGH && buttonState == LOW) {
//    delay(20);
//    if (digitalRead(BUTTON_A_PIN) == LOW) {
//      if (logReadyToDump) {
//        Serial.println(F("i,x_mm,y_mm,ts"));
//        for (int i = 0; i < logIndex; i++) {
//          float x = logBuffer[i].x_centi / 100.0f;
//          float y = logBuffer[i].y_centi / 100.0f - 300.0f;
//          Serial.print(i);
//          Serial.print(",");
//          Serial.print(x);
//          Serial.print(",");
//          Serial.print(y);
//          Serial.print(",");
//          Serial.println(logBuffer[i].t_ms);
//        }
//        logReadyToDump = false;
//        lastButtonState = buttonState;
//        return;
//      }
//
//      if (robotState == WAITING_FOR_TARGET_BUTTON) {
//        startTargetSampling();
//        robotState = SAMPLING_TARGET;
//      }
//    }
//  }
//
//  lastButtonState = buttonState;
//
//  switch (robotState) {
//    case WAITING_FOR_TARGET_BUTTON:
//      stateWaitingForButton();
//      break;
//    case SAMPLING_TARGET:
//      stateSampling();
//      break;
//    case WAITING_FOR_LEADER_OFF:
//      stateWaitingForLeaderOff();
//      break;
//    case WAITING_FOR_LEADER_ON:
//      stateWaitingForLeaderOn();
//      break;
//    case FOLLOWING:
//      stateFollowing();
//      break;
//    case STOPPED:
//      stateStopped();
//      break;
//  }
//
//  logData();
//  plotDebugSignals();
//}

#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "Kinematics.h"

Motors_c motors;
LineSensors_c line_sensors;
PID_c left_pid;
PID_c right_pid;
Kinematics_c pose;

//==========================================================================//
// TIMING
//==========================================================================//

uint32_t lastLoop = 0;
const uint32_t LOOP_TIME_MS = 10;

unsigned long speed_est_ts = 0;
unsigned long pose_ts = 0;
const uint32_t SPEED_EST_MS = 10;
const uint32_t POSE_MS = 10;

//==========================================================================//
// SPEED ESTIMATION
//==========================================================================//

const float ALPHA = 0.2f;

long last_e0 = 0;
long last_e1 = 0;

float right_speed = 0.0f;
float left_speed = 0.0f;
float last_speed_e0 = 0.0f;
float last_speed_e1 = 0.0f;

//==========================================================================//
// LINE SENSOR BASELINE / SIGNAL
//==========================================================================//

float line_baseline[NUM_SENSORS];
float baselineTotal = 0.0f;
float leaderThreshold = 0.0f;
float leaderOffThreshold = 0.0f;

float total = 0.0f;
float filteredTotal = 0.0f;
float prevTotal = 0.0f;
float L = 0.0f;
float C = 0.0f;
float R = 0.0f;
float smoothedLR = 0.0f;

//==========================================================================//
// TARGET SAMPLING
//==========================================================================//

const uint32_t TARGET_SAMPLE_TIME_MS = 2000;
const uint32_t TARGET_SETTLE_MS = 300;

uint32_t targetSampleStart = 0;
float targetSampleSum = 0.0f;
int targetSampleCount = 0;
float minTotalSeen = 100000.0f;
float maxTotalSeen = -100000.0f;

float targetTotal = 0.0f;
float targetLR = 0.0f;
float lrSampleSum = 0.0f;
float lowerThreshold = 0.0f;
float upperThreshold = 0.0f;

const float BAND_MARGIN_FRACTION = 0.20f;
const float MIN_BAND_MARGIN = 15.0f;

//==========================================================================//
// MOVEMENT CONTROL
//==========================================================================//

float smoothedDrive = 0.0f;

const float MAX_FWD_SPEED_DEMAND = 0.20f;
const float SPEED_GAIN = 0.2f;
const float TARGET_DEADBAND = 2.4f;

const float STEER_GAIN = 0.0005f;
const float MAX_STEER = 0.15f;

const int PWM_MAX_ABS = 60;
const int PWM_FLOOR_FWD = 18;

const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

// no-turn startup window
const uint32_t NO_TURN_AFTER_START_MS = 1000;
uint32_t followMotionStartTime = 0;
bool hasStartedFollowingMotion = false;
const float FOLLOW_START_THRESHOLD = 0.03f;

// leader lost timeout
const uint32_t LOST_TIMEOUT_MS = 800;
uint32_t leaderLostStart = 0;

// wait-off / wait-on persistence
const uint32_t LEADER_OFF_CONFIRM_MS = 400;
const uint32_t LEADER_ON_CONFIRM_MS  = 150;
uint32_t leaderOffCandidateStart = 0;
uint32_t leaderOnCandidateStart  = 0;

//==========================================================================//
// FSM
//==========================================================================//

enum RobotState {
  WAITING_FOR_TARGET_BUTTON,
  SAMPLING_TARGET,
  WAITING_FOR_LEADER_OFF,
  WAITING_FOR_LEADER_ON,
  FOLLOWING,
  STOPPED
};

RobotState robotState = WAITING_FOR_TARGET_BUTTON;

//==========================================================================//
// LOGGING
//==========================================================================//

const uint32_t LOG_MS = 50;
const int LOG_SIZE = 200;

struct LogEntry {
  uint32_t t_ms;
  int16_t x_centi;
  int16_t y_centi;
};

LogEntry logBuffer[LOG_SIZE];
int logIndex = 0;
uint32_t lastLogTime = 0;
bool loggingActive = false;
bool logReadyToDump = false;

const int BUTTON_A_PIN = 14;
bool lastButtonState = HIGH;

//==========================================================================//
// SERIAL PLOTTER
//==========================================================================//

const bool USE_SERIAL_PLOTTER = true;
uint32_t plot_ts = 0;
const uint32_t PLOT_MS = 50;

//==========================================================================//
// HELPERS
//==========================================================================//

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
  smoothedDrive = 0.0f;

  if (loggingActive) {
    loggingActive = false;
    logReadyToDump = true;
  }
}

void logData() {
  if (!loggingActive) return;

  uint32_t now = millis();
  if (now - lastLogTime < LOG_MS) return;
  lastLogTime = now;

  if (logIndex >= LOG_SIZE) return;

  logBuffer[logIndex].x_centi = (int16_t)(pose.x * 100.0f);
  logBuffer[logIndex].y_centi = (int16_t)((pose.y + 300.0f) * 100.0f);
  logBuffer[logIndex].t_ms = now;
  logIndex++;
}

//==========================================================================//
// SPEED / POSE
//==========================================================================//

void speedcalc(unsigned long elapsed_time) {
  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;

  last_e0 = count_e0;
  last_e1 = count_e1;

  float speed_e0 = float(delta_e0) / float(elapsed_time);
  float speed_e1 = float(delta_e1) / float(elapsed_time);

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

void updatePoseIfNeeded() {
  unsigned long now = millis();
  if (now - pose_ts >= POSE_MS) {
    pose_ts = now;
    pose.update();
  }
}

//==========================================================================//
// LINE SENSOR CONVERSION
//==========================================================================//

float readingToSignal(float reading, float baseline) {
  if (baseline <= 1.0f) return 0.0f;

  float x = 100.0f * (baseline - reading) / baseline;

  if (x < 0.0f) x = 0.0f;
  if (x > 100.0f) x = 100.0f;
  return x;
}

void updateSensors() {
  line_sensors.readSensorsDigital();

  float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
  float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
  float s2 = readingToSignal(line_sensors.readings[2], line_baseline[2]);
  float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
  float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);

  L = s0 + s1;
  C = s2;
  R = s3 + s4;

  prevTotal = total;

  // More stable than L+C+R for this use
  total = L + 0.5*C + R;

  // Filter only for threshold / state decisions
  filteredTotal = 0.7f * filteredTotal + 0.3f * total;
}

void measureLineBaseline() {
  const int N = 80;

  for (int i = 0; i < NUM_SENSORS; i++) {
    line_baseline[i] = 0.0f;
  }

  for (int n = 0; n < N; n++) {
    line_sensors.readSensorsDigital();
    for (int i = 0; i < NUM_SENSORS; i++) {
      line_baseline[i] += line_sensors.readings[i];
    }
    delay(5);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    line_baseline[i] /= (float)N;
  }

  baselineTotal = 0.0f;
  for (int n = 0; n < 50; n++) {
    line_sensors.readSensorsDigital();


    float s0 = readingToSignal(line_sensors.readings[0], line_baseline[0]);
    float s1 = readingToSignal(line_sensors.readings[1], line_baseline[1]);
    float s3 = readingToSignal(line_sensors.readings[3], line_baseline[3]);
    float s4 = readingToSignal(line_sensors.readings[4], line_baseline[4]);

    baselineTotal += (s0 + s1 + s3 + s4);
    delay(5);


  }

  baselineTotal /= 50.0f;
  filteredTotal = baselineTotal;
}

//==========================================================================//
// TARGET SAMPLING
//==========================================================================//

void startTargetSampling() {
  targetSampleStart = millis();
  targetSampleSum = 0.0f;
  targetSampleCount = 0;
  lrSampleSum = 0.0f;
  minTotalSeen = 100000.0f;
  maxTotalSeen = -100000.0f;
  total = 0.0f;
  filteredTotal = baselineTotal;
}

void finishTargetSampling() {
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = targetSampleSum / (float)targetSampleCount;
  targetLR = lrSampleSum / (float)targetSampleCount;

  leaderThreshold = baselineTotal + 0.4f * (targetTotal - baselineTotal);
  leaderOffThreshold = baselineTotal + 0.1f * (targetTotal - baselineTotal);

  float observedSpread = maxTotalSeen - minTotalSeen;
  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  Serial.println(F("target sampling done"));
  Serial.print(F("targetTotal,")); Serial.println(targetTotal);
  Serial.print(F("lowerThreshold,")); Serial.println(lowerThreshold);
  Serial.print(F("upperThreshold,")); Serial.println(upperThreshold);
  Serial.print(F("leaderThreshold,")); Serial.println(leaderThreshold);
  Serial.print(F("leaderOffThreshold,")); Serial.println(leaderOffThreshold);
}

//==========================================================================//
// STATE FUNCTIONS
//==========================================================================//

void stateWaitingForButton() {
  motors.setPWM(0, 0);
}

void stateSampling() {
  motors.setPWM(0, 0);

  if ((millis() - targetSampleStart) > TARGET_SETTLE_MS && filteredTotal > leaderThreshold) {
    targetSampleSum += total;
    lrSampleSum += (L - R);
    targetSampleCount++;


    if (total < minTotalSeen) minTotalSeen = total;
    if (total > maxTotalSeen) maxTotalSeen = total;


  }

  if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
    if (targetSampleCount < 20) {
      robotState = WAITING_FOR_TARGET_BUTTON;
      return;
    }
    finishTargetSampling();
    leaderOffCandidateStart = 0;
    robotState = WAITING_FOR_LEADER_OFF;
  }
}

void stateWaitingForLeaderOff() {
  motors.setPWM(0, 0);

  if (filteredTotal < leaderOffThreshold) {
    if (leaderOffCandidateStart == 0) {
      leaderOffCandidateStart = millis();
    }


    if (millis() - leaderOffCandidateStart >= LEADER_OFF_CONFIRM_MS) {
      total = 0.0f;
      filteredTotal = baselineTotal;
      smoothedLR = 0.0f;
      leaderOnCandidateStart = 0;
      robotState = WAITING_FOR_LEADER_ON;
    }


  } else {
    leaderOffCandidateStart = 0;
  }
}

void stateWaitingForLeaderOn() {
  motors.setPWM(0, 0);

  if (filteredTotal > leaderThreshold) {

    left_pid.reset();
    right_pid.reset();
    smoothedDrive = 0.0f;
    smoothedLR = (L - R) - targetLR;
    hasStartedFollowingMotion = false;
    followMotionStartTime = 0;
    leaderLostStart = 0;

    logIndex = 0;
    loggingActive = true;
    logReadyToDump = false;
    lastLogTime = millis() - LOG_MS;

    robotState = FOLLOWING;
  }



}

void stateFollowing() {
  if (!loggingActive && !logReadyToDump) {
    logIndex = 0;
    loggingActive = true;
    lastLogTime = millis() - LOG_MS;
  }

  float error = (targetTotal - TARGET_DEADBAND) - total;
  float desiredDrive = 0.0f;

  if (total < targetTotal - TARGET_DEADBAND) {
    desiredDrive = SPEED_GAIN * error;
  }

  desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);

  if (filteredTotal < leaderOffThreshold) {
    if (leaderLostStart == 0) leaderLostStart = millis();


    if (millis() - leaderLostStart > LOST_TIMEOUT_MS) {
      stopRobot();
      robotState = STOPPED;
      return;
    }

    // hold last drive briefly while confirming real loss
    desiredDrive = smoothedDrive;


  } else {
    leaderLostStart = 0;
  }

  smoothedDrive = 0.3f * desiredDrive + 0.7f * smoothedDrive;

  if (smoothedDrive > FOLLOW_START_THRESHOLD) {
    if (!hasStartedFollowingMotion) {
      hasStartedFollowingMotion = true;
      followMotionStartTime = millis();
    }
  }

  float rawLRDiff = (L - R) - targetLR;
  smoothedLR = 0.5f * rawLRDiff + 0.5f * smoothedLR;

  bool steeringEnabled = true;
  if (hasStartedFollowingMotion) {
    if (millis() - followMotionStartTime < NO_TURN_AFTER_START_MS) {
      steeringEnabled = false;
    }
  }

  float steer = 0.0f;
  if (steeringEnabled) {
    steer = clampFloat(-STEER_GAIN * smoothedLR, -MAX_STEER, MAX_STEER);
  }

  float l_demand = smoothedDrive + steer;
  float r_demand = smoothedDrive - steer;

  float l_pwm_f = left_pid.update(l_demand, left_speed);
  float r_pwm_f = right_pid.update(r_demand, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  l_pwm = applyPwmFloor(l_pwm, l_demand, PWM_FLOOR_FWD);
  r_pwm = applyPwmFloor(r_pwm, r_demand, PWM_FLOOR_FWD);

  motors.setPWM(l_pwm, r_pwm);
}

void stateStopped() {
  motors.setPWM(0, 0);
}

//==========================================================================//
// SERIAL PLOTTER
//==========================================================================//

//void plotDebugSignals() {
//  if (!USE_SERIAL_PLOTTER) return;
//
//  uint32_t now = millis();
//  if (now - plot_ts < PLOT_MS) return;
//  plot_ts = now;
//
//  float driveGate = (total < targetTotal - TARGET_DEADBAND) ? 100.0f : 0.0f;
//  float lostGate  = (filteredTotal < leaderOffThreshold) ? 100.0f : 0.0f;
//
//  Serial.print(total, 2);               Serial.print(",");
//  Serial.print(filteredTotal, 2);       Serial.print(",");
//  Serial.print(targetTotal, 2);         Serial.print(",");
//  Serial.print(leaderThreshold, 2);     Serial.print(",");
//  Serial.print(leaderOffThreshold, 2);  Serial.print(",");
//  Serial.print(100 * smoothedDrive, 2); Serial.print(",");
//  Serial.print(driveGate, 1);           Serial.print(",");
//  Serial.print(lostGate, 1);            Serial.print(",");
//  Serial.println((int)robotState);
//}

//==========================================================================//
// SETUP
//==========================================================================//

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  motors.initialise();
  line_sensors.initialiseForDigital();
  line_sensors.timeout_us = 8000;
  setupEncoder0();
  setupEncoder1();

  pinMode(EMIT_PIN, INPUT);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();

  pose.initialise(0.0f, 0.0f, 0.0f);

  last_e0 = count_e0;
  last_e1 = count_e1;
  speed_est_ts = millis();
  pose_ts = millis();

  measureLineBaseline();

  robotState = WAITING_FOR_TARGET_BUTTON;
  smoothedDrive = 0.0f;
  hasStartedFollowingMotion = false;
  followMotionStartTime = 0;
  leaderLostStart = 0;
  leaderOffCandidateStart = 0;
  leaderOnCandidateStart = 0;

  lastLoop = millis();
}

//==========================================================================//
// LOOP
//==========================================================================//

void loop() {
  updateWheelSpeedsIfNeeded();
  updatePoseIfNeeded();

  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  updateSensors();

  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (lastButtonState == HIGH && buttonState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW) {
      if (logReadyToDump) {
        Serial.println(F("i,x_mm,y_mm,ts"));
        for (int i = 0; i < logIndex; i++) {
          float x = logBuffer[i].x_centi / 100.0f;
          float y = logBuffer[i].y_centi / 100.0f - 300.0f;
          Serial.print(i);
          Serial.print(",");
          Serial.print(x);
          Serial.print(",");
          Serial.print(y);
          Serial.print(",");
          Serial.println(logBuffer[i].t_ms);
        }
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

  lastButtonState = buttonState;

  switch (robotState) {
    case WAITING_FOR_TARGET_BUTTON:
      stateWaitingForButton();
      break;
    case SAMPLING_TARGET:
      stateSampling();
      break;
    case WAITING_FOR_LEADER_OFF:
      stateWaitingForLeaderOff();
      break;
    case WAITING_FOR_LEADER_ON:
      stateWaitingForLeaderOn();
      break;
    case FOLLOWING:
      stateFollowing();
      break;
    case STOPPED:
      stateStopped();
      break;
  }

  logData();
//  plotDebugSignals();
}
