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

// timing
uint32_t lastLoop = 0;
const int LOOP_TIME_MS = 20;

// speed / pose estimation timing
unsigned long speed_est_ts = 0;
unsigned long pose_ts = 0;
const int SPEED_EST_MS = 10;
const int POSE_MS = 10;
const float ALPHA = 0.2f;

// encoder history for speed estimation
long last_e0 = 0;
long last_e1 = 0;

// calibration spin
const float CAL_TURN_DEMAND = 0.32f;         // wheel speed demand for spin
const float CAL_ANGLE_TARGET = 4.0f * PI;    // 720 degrees

// wait after calibration before target sampling
const int POST_CAL_WAIT_MS = 1000;

// target sampling after calibration
const int TARGET_SAMPLE_TIME_MS = 2000;

// straight-line follow using encoder speed demand
const float MAX_FWD_SPEED_DEMAND = 0.35f;    // matched to leader
const float SPEED_GAIN = 0.01f;
const float TARGET_DEADBAND = 2.5f;

// threshold tuning
const float BAND_MARGIN_FRACTION = 0.20f;
const float MIN_BAND_MARGIN = 15.0f;

// pwm limits / floor
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR_FWD = 18;
const int PWM_FLOOR_TURN = 20;

// pid gains
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

// grouped values
float total = 0;
float L = 0;
float R = 0;
float C = 0;

// target values
float targetTotal = 0;
float lowerThreshold = 0;
float upperThreshold = 0;

// forward speed demand
float smoothedDrive = 0;

// measured wheel speeds
float right_speed = 0;
float left_speed = 0;
float last_speed_e0 = 0;
float last_speed_e1 = 0;

// calibration heading tracking
float cal_start_theta = 0.0f;
float cal_accumulated_turn = 0.0f;
float cal_last_theta = 0.0f;

// state
enum RobotState {
  CALIBRATING_SPIN,
  WAITING_AFTER_CAL,
  SAMPLING_TARGET,
  FOLLOWING
};

RobotState state = CALIBRATING_SPIN;
uint32_t waitAfterCalStart = 0;
uint32_t targetSampleStart = 0;
float targetSampleSum = 0;
int targetSampleCount = 0;
float minTotalSeen = 100000.0f;
float maxTotalSeen = -100000.0f;

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

void stopRobot()
{
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

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

  total = L + C + R;
}

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

void startTargetSampling()
{
  targetSampleStart = millis();
  targetSampleSum = 0;
  targetSampleCount = 0;
  minTotalSeen = 100000.0f;
  maxTotalSeen = -100000.0f;
}

void finishTargetSampling()
{
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = targetSampleSum / (float)targetSampleCount;

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

void setup()
{
  Serial.begin(115200);

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

  delay(500);
  lastLoop = millis();
}

void loop()
{
  updateWheelSpeedsIfNeeded();
  updatePoseIfNeeded();

  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  if (state == SAMPLING_TARGET || state == FOLLOWING) {
    updateSensors();
  }

  if (state == CALIBRATING_SPIN) {
    if (updateCalibrationSpin()) {
      state = WAITING_AFTER_CAL;
      waitAfterCalStart = millis();
      stopRobot();
    }
  }
  else if (state == WAITING_AFTER_CAL) {
    stopRobot();

    if (millis() - waitAfterCalStart >= POST_CAL_WAIT_MS) {
      state = SAMPLING_TARGET;
      startTargetSampling();
    }
  }
  else if (state == SAMPLING_TARGET) {
    stopRobot();

    targetSampleSum += total;
    targetSampleCount++;

    if (total < minTotalSeen) minTotalSeen = total;
    if (total > maxTotalSeen) maxTotalSeen = total;

    if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
      finishTargetSampling();
      state = FOLLOWING;
      smoothedDrive = 0;
      left_pid.reset();
      right_pid.reset();
    }
  }
  else if (state == FOLLOWING) {
    float desiredDrive = 0.0f;

    if (total > targetTotal + TARGET_DEADBAND) {
      desiredDrive = SPEED_GAIN * (total - targetTotal - TARGET_DEADBAND);
    } else {
      desiredDrive = 0.0f;
    }

    desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);

    // no smoothing
    smoothedDrive = desiredDrive;

    float l_pwm_f = left_pid.update(smoothedDrive, left_speed);
    float r_pwm_f = right_pid.update(smoothedDrive, right_speed);

    int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
    int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

    l_pwm = applyPwmFloor(l_pwm, smoothedDrive, PWM_FLOOR_FWD);
    r_pwm = applyPwmFloor(r_pwm, smoothedDrive, PWM_FLOOR_FWD);

    motors.setPWM(l_pwm, r_pwm);
  }

  Serial.print(total);
  Serial.print(",");
  Serial.print(targetTotal);
  Serial.print(",");
  Serial.print(lowerThreshold);
  Serial.print(",");
  Serial.print(upperThreshold);
  Serial.print(",");
  Serial.print(smoothedDrive);
  Serial.print(",");
  Serial.print(left_speed);
  Serial.print(",");
  Serial.print(right_speed);
  Serial.print(",");
  Serial.print(pose.theta);
  Serial.print(",");
  Serial.print(cal_accumulated_turn);
  Serial.print(",");
  Serial.println((int)state);
}
