//==========================================================================//
//                        FOLLOWER: USE BLACK SCREEN ROBOT                  //
//==========================================================================//
//==========================================================================//
//                        SECTION A: VARIABLES                              //
//==========================================================================//

#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "Kinematics.h"
#include <math.h>

Motors_c motors;
LineSensors_c line_sensors;
PID_c left_pid;
PID_c right_pid;
Kinematics_c pose;

//==========================================================================//
//                        2. TIMING INTERVALS                               //
//==========================================================================//

uint32_t lastLoop = 0;
const int LOOP_TIME_MS = 10;

// speed / pose estimation timing
unsigned long speed_est_ts = 0;
unsigned long pose_ts = 0;
const int SPEED_EST_MS = 10;
const int POSE_MS = 10;

//==========================================================================//
//                        3. WHEEL SPEED EST VARIABLES                      //
//==========================================================================//

const float ALPHA = 0.2f;

// encoder history for speed estimation
long last_e0 = 0;
long last_e1 = 0;

// measured wheel speeds
float right_speed = 0;
float left_speed = 0;
float last_speed_e0 = 0;
float last_speed_e1 = 0;

//==========================================================================//
//                        4. CAL SPIN VARIABLES                             //
//==========================================================================//

const float CAL_TURN_DEMAND = 0.32f;
const float CAL_ANGLE_TARGET = 4.0f * PI;

// wait after calibration before target sampling
const int POST_CAL_WAIT_MS = 5000;

// calibration heading tracking
float cal_start_theta = 0.0f;
float cal_accumulated_turn = 0.0f;
float cal_last_theta = 0.0f;

//==========================================================================//
//                        5. TARGET SAMPLING VARIABLES                      //
//==========================================================================//

const int TARGET_SAMPLE_TIME_MS = 2000;

uint32_t targetSampleStart = 0;
float targetSampleSum = 0;
int targetSampleCount = 0;
float minTotalSeen = 100000.0f;
float maxTotalSeen = -100000.0f;

// target values
float targetTotal = 0;
float targetLR = 0.0f;
float lrSampleSum = 0.0f;
float lowerThreshold = 0;
float upperThreshold = 0;

// threshold tuning
const float BAND_MARGIN_FRACTION = 0.20f;
const float MIN_BAND_MARGIN = 15.0f;

//==========================================================================//
//                        6. PID/MOVEMENT CONTROL VARIABLES                 //
//==========================================================================//

float smoothedDrive = 0;

// straight-line follow using encoder speed demand
const float MAX_FWD_SPEED_DEMAND = 0.35f;
const float SPEED_GAIN = 0.01f;
const float TARGET_DEADBAND = 2.0f;

// steering
const float STEER_GAIN = 0.0045f;
const float STEER_GAIN_SCALE = 0.00001f;  // unused, kept from your version
const float MAX_STEER = 0.25f;

// pwm limits / floor
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR_FWD = 18;
const int PWM_FLOOR_TURN = 20;

// pid gains
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

// stop / latch logic
const uint32_t STOP_SETTLE_MS = 1000;
uint32_t belowDriveSince = 0;
bool hasStartedFollowingMotion = false;
const float FOLLOW_START_THRESHOLD = 0.03f;

//==========================================================================//
//                        7. SENSOR GROUPS                                  //
//==========================================================================//

float total = 0;
float prevTotal = 0;
float L = 0;
float R = 0;
float C = 0;
float smoothedLR = 0.0f;

//==========================================================================//
//                        8. FSM SETTING                                    //
//==========================================================================//

enum RobotState {
  CALIBRATING_SPIN,
  WAITING_AFTER_CAL,
  SAMPLING_TARGET,
  FOLLOWING,
  STOPPED
};

RobotState state = CALIBRATING_SPIN;
uint32_t waitAfterCalStart = 0;

//==========================================================================//
//                        9. LOGGING                                        //
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

//==========================================================================//
//                        10. BUZZER                                        //
//==========================================================================//

const int BUZZER_PIN = 6;
uint32_t beepStopTime = 0;

const uint32_t SAMPLE_BEEP_MS = 80;
const uint32_t STOPPED_BEEP_MS = 120;
const int BUZZER_PWM = 120;

//==========================================================================//
//                        SECTION B: HELPER FUNCTIONS                       //
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
  if (fabs(demand_speed) < 1e-6f) return 0;
  if (abs(pwm) < floorVal) {
    pwm = (demand_speed > 0.0f) ? floorVal : -floorVal;
  }
  return pwm;
}

float angleDiff(float target, float source) {
  return atan2f(sinf(target - source), cosf(target - source));
}

void setBeep(uint32_t durationMs) {
  analogWrite(BUZZER_PIN, BUZZER_PWM);
  beepStopTime = millis() + durationMs;
}

void updateBeep() {
  if (beepStopTime != 0 && millis() >= beepStopTime) {
    analogWrite(BUZZER_PIN, 0);
    beepStopTime = 0;
  }
}

void stopRobot()
{
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
  smoothedDrive = 0;

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

void LogDump() {
  bool buttonState = digitalRead(BUTTON_A_PIN);

  static bool lastBtn = HIGH;
  if (lastBtn == HIGH && buttonState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW && logReadyToDump) {
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
    }
  }
  lastBtn = buttonState;
}

//==========================================================================//
//                        SECTION C: KEY FUNCTIONS                          //
//==========================================================================//

void speedcalc(unsigned long elapsed_time)
{
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

void updateWheelSpeedsIfNeeded()
{
  unsigned long elapsed_time = millis() - speed_est_ts;
  if (elapsed_time >= SPEED_EST_MS) {
    speedcalc(elapsed_time);
    speed_est_ts = millis();
  }
}

void updatePoseIfNeeded()
{
  unsigned long now = millis();
  if (now - pose_ts >= POSE_MS) {
    pose_ts = now;
    pose.update();
  }
}

void updateSensors()
{
  line_sensors.readSensorsADC();

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = line_sensors.readings[i];
    int minv = line_sensors.minimum[i];
    int range = line_sensors.range[i];

    if (range < 1) range = 1;

    float norm = (float)(raw - minv) / (float)range;
    norm = clampFloat(norm, 0.0f, 1.0f);

    line_sensors.readings[i] = norm * 100.0f;
  }

  L = line_sensors.readings[0] + line_sensors.readings[1];
  C = line_sensors.readings[2];
  R = line_sensors.readings[3] + line_sensors.readings[4];

  prevTotal = total;
  total = L + C + R;
}

//==========================================================================//
//                        CALIBRATION                                       //
//==========================================================================//

void startCalibrationSpin()
{
  pinMode(EMIT_PIN, INPUT);

  for (int i = 0; i < NUM_SENSORS; i++) {
    line_sensors.minimum[i] = 1023;
    line_sensors.maximum[i] = 0;
    line_sensors.range[i] = 1;
  }

  left_pid.reset();
  right_pid.reset();

  cal_start_theta = pose.theta;
  cal_last_theta = pose.theta;
  cal_accumulated_turn = 0.0f;
}

bool updateCalibrationSpin()
{
  line_sensors.readSensorsADC();
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = line_sensors.readings[i];
    if (v < line_sensors.minimum[i]) line_sensors.minimum[i] = v;
    if (v > line_sensors.maximum[i]) line_sensors.maximum[i] = v;
  }

  float dtheta = angleDiff(pose.theta, cal_last_theta);
  cal_accumulated_turn += fabs(dtheta);
  cal_last_theta = pose.theta;

  float demand_L = -CAL_TURN_DEMAND;
  float demand_R = +CAL_TURN_DEMAND;

  float l_pwm_f = left_pid.update(demand_L, left_speed);
  float r_pwm_f = right_pid.update(demand_R, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  l_pwm = applyPwmFloor(l_pwm, demand_L, PWM_FLOOR_TURN);
  r_pwm = applyPwmFloor(r_pwm, demand_R, PWM_FLOOR_TURN);

  motors.setPWM(l_pwm, r_pwm);

  if (cal_accumulated_turn >= CAL_ANGLE_TARGET) {
    motors.setPWM(0, 0);

    for (int i = 0; i < NUM_SENSORS; i++) {
      int r = line_sensors.maximum[i] - line_sensors.minimum[i];
      if (r < 20) r = 20;
      line_sensors.range[i] = r;
    }

    Serial.println("sensor calibration done");
    return true;
  }

  return false;
}

//==========================================================================//
//                        TARGET SAMPLING                                   //
//==========================================================================//

void startTargetSampling()
{
  targetSampleStart = millis();
  targetSampleSum = 0;
  targetSampleCount = 0;
  lrSampleSum = 0.0f;
  minTotalSeen = 100000.0f;
  maxTotalSeen = -100000.0f;

  setBeep(SAMPLE_BEEP_MS);
}

void finishTargetSampling()
{
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = targetSampleSum / (float)targetSampleCount;
  targetLR = lrSampleSum / (float)targetSampleCount;

  float observedSpread = maxTotalSeen - minTotalSeen;
  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  Serial.println("target sampling done");
  Serial.print("targetTotal,"); Serial.println(targetTotal);
  Serial.print("lowerThreshold,"); Serial.println(lowerThreshold);
  Serial.print("upperThreshold,"); Serial.println(upperThreshold);
}

//==========================================================================//
//                        STATE FUNCTIONS                                   //
//==========================================================================//

void stateCalibratingSpin() {
  if (updateCalibrationSpin()) {
    state = WAITING_AFTER_CAL;
    waitAfterCalStart = millis();
    stopRobot();
  }
}

void stateWaiting() {
  stopRobot();
  if (millis() - waitAfterCalStart >= POST_CAL_WAIT_MS) {
    state = SAMPLING_TARGET;
    startTargetSampling();
  }
}

void stateSampling() {
  stopRobot();

  targetSampleSum += total;
  lrSampleSum += (L - R);
  targetSampleCount++;

  if (total < minTotalSeen) minTotalSeen = total;
  if (total > maxTotalSeen) maxTotalSeen = total;

  if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
    finishTargetSampling();
    state = FOLLOWING;
    smoothedDrive = 0;
    smoothedLR = (L - R) - targetLR;
    belowDriveSince = 0;
    hasStartedFollowingMotion = false;
    left_pid.reset();
    right_pid.reset();
  }
}

void stateFollowing() {

  if (!loggingActive && !logReadyToDump) {
    logIndex = 0;
    loggingActive = true;
    lastLogTime = millis() - LOG_MS;
  }

  float error = total - (targetTotal + TARGET_DEADBAND);
  float desiredDrive = 0.0f;

  if (total > targetTotal + TARGET_DEADBAND) {
    desiredDrive = SPEED_GAIN * error;
  }
  desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);

  // Smoothing
  smoothedDrive = 0.3f * desiredDrive + 0.7f * smoothedDrive;

  if (smoothedDrive > FOLLOW_START_THRESHOLD) {
    hasStartedFollowingMotion = true;
  }

  if (smoothedDrive < 0.01f) {
    if (hasStartedFollowingMotion) {
      if (belowDriveSince == 0) {
        belowDriveSince = millis();
      }

      motors.setPWM(0, 0);

      if (millis() - belowDriveSince >= STOP_SETTLE_MS) {
        stopRobot();
        state = STOPPED;
        setBeep(STOPPED_BEEP_MS);
      }
    } else {
      belowDriveSince = 0;
      motors.setPWM(0, 0);
    }
    return;
  } else {
    belowDriveSince = 0;
  }

  float rawLRDiff = (L - R) - targetLR;
  smoothedLR = 0.5f * rawLRDiff + 0.5f * smoothedLR;

  float steer = clampFloat(STEER_GAIN * smoothedLR, -MAX_STEER, MAX_STEER);

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
//                        SETUP                                             //
//==========================================================================//

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 0);

  motors.initialise();
  line_sensors.initialiseForADC();
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

  delay(500);
  startCalibrationSpin();

  state = CALIBRATING_SPIN;
  waitAfterCalStart = 0;
  smoothedDrive = 0;
  belowDriveSince = 0;
  hasStartedFollowingMotion = false;

  delay(500);
  lastLoop = millis();
}

//==========================================================================//
//                        LOOP                                              //
//==========================================================================//

void loop()
{
  updateWheelSpeedsIfNeeded();
  updatePoseIfNeeded();
  updateBeep();

  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  if (state == SAMPLING_TARGET || state == FOLLOWING) {
    updateSensors();
  }

  switch (state) {
    case CALIBRATING_SPIN:
      stateCalibratingSpin();
      break;
    case WAITING_AFTER_CAL:
      stateWaiting();
      break;
    case SAMPLING_TARGET:
      stateSampling();
      break;
    case FOLLOWING:
      stateFollowing();
      break;
    case STOPPED:
      stateStopped();
      break;
  }

  logData();
  LogDump();

//  Serial.print(total);
//  Serial.print(",");
//  Serial.print(targetTotal);
//  Serial.print(",");
//  Serial.print(lowerThreshold);
//  Serial.print(",");
//  Serial.print(upperThreshold);
//  Serial.print(",");
//  Serial.print(smoothedDrive);
//  Serial.print(",");
//  Serial.print(left_speed);
//  Serial.print(",");
//  Serial.print(right_speed);
//  Serial.print(",");
//  Serial.print(pose.theta);
//  Serial.print(",");
//  Serial.print(cal_accumulated_turn);
//  Serial.print(",");
//  Serial.println((int)state);
}
