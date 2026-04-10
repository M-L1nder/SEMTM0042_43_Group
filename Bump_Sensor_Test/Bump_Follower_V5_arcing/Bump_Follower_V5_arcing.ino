#include "Motors.h"
#include "BumpSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "oled.h"
#include "Kinematics.h"

Kinematics_c pose;
OLED_c display(1, 30, 0, 17, 13);   // clk, mosi, rst, dc, cs

const bool USE_OLED = true;

unsigned long oled_ts = 0;
#define OLED_UPDATE_MS 400
unsigned long pose_ts = 0;
const uint32_t POSE_MS = 20;

Motors_c motors;
BumpSensors_c bump_sensors;
PID_c left_pid;
PID_c right_pid;

// -------------------- Logging -------------------
const uint32_t LOG_MS = 50;
const int LOG_SIZE = 100;

struct LogEntry {
  uint16_t x_centi;
  uint16_t y_centi;
  uint32_t t_ms;
};

LogEntry logBuffer[LOG_SIZE];
int logIndex = 0;
uint32_t lastLogTime = 0;
bool loggingActive = false;
bool logReadyToDump = false;

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
float bearingSignal = 0.0f;   // filtered (L-R)/(L+R)

// baseline raw discharge times (beacon OFF)
float bump_baseline[BUMP_NUM_SENSORS];
float baselineTotal = 0.0f;
float beaconThreshold = 0.0f;

// -------------------- TARGET SAMPLE --------------------
const uint32_t TARGET_SETTLE_MS = 300;
float targetTotal = 0.0f;
float targetBearing = 0.0f;   // learned from sampled pose
float lowerThreshold = 0.0f;
float upperThreshold = 0.0f;

float targetSampleSum = 0.0f;
float targetBearingSum = 0.0f;
int targetSampleCount = 0;
float minTotalSeen = 100000.0f;
float maxTotalSeen = -100000.0f;

// -------------------- TUNING --------------------
const float TARGET_DEADBAND = 2.0f;
const uint32_t TARGET_SAMPLE_TIME_MS = 2000;
const uint32_t LOST_TIMEOUT_MS = 200;

// longitudinal control
const float MAX_FWD_SPEED_DEMAND = 0.45f;
const float SPEED_GAIN = 0.4f;

// lateral / bearing control
const float STEER_GAIN = 0.25f;
const float MAX_STEER_DEMAND = 0.035f;
const float BEARING_DEADBAND = 0.05f;
const float BEARING_EPS = 0.5f;
const float BEARING_FILTER_ALPHA = 0.3f;
const float STEER_ENABLE_TOTAL_FRAC = 0.50f;

// threshold band built from observed target variation
const float BAND_MARGIN_FRACTION = 0.10f;
const float MIN_BAND_MARGIN = 3.0f;

// pwm limits / floor
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR_FWD = 18;

// pid gains
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

// if follower drives backwards instead of forwards, flip this
const float FOLLOW_SIGN = 1.0f;

// -------------------- STATE --------------------
enum RobotState {
  WAITING_FOR_TARGET_BUTTON,
  SAMPLING_TARGET,
  WAITING_FOR_BEACON_OFF,
  WAITING_FOR_BEACON_ON,
  FOLLOWING
};

RobotState robotstate = WAITING_FOR_TARGET_BUTTON;

bool lastButtonState = HIGH;
uint32_t targetSampleStart = 0;
uint32_t beaconLostStart = 0;
float smoothedDrive = 0.0f;

// -------------------- HELPERS --------------------
void logPoseIfNeeded() {
  if (!loggingActive) return;

  uint32_t now = millis();
  if (now - lastLogTime < LOG_MS) return;
  lastLogTime = now;

  if (logIndex >= LOG_SIZE) return;

  logBuffer[logIndex].x_centi = (uint16_t)((pose.x) * 100.0f);
  logBuffer[logIndex].y_centi = (uint16_t)((pose.y + 300.0f) * 100.0f);
  logBuffer[logIndex].t_ms = millis();
  logIndex++;
}

void updatePoseIfNeeded() {
  unsigned long now = millis();
  if (now - pose_ts >= POSE_MS) {
    pose_ts = now;
    pose.update();
  }
}

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

void drawFollowerOLED()
{
  display.clear();
  display.gotoXY(0, 0);
  display.print(getStateName(robotstate));
  display.gotoXY(0, 1);
  display.print("T:");
  display.print(total, 1);
  display.print(" B:");
  display.print(bearingSignal, 2);
}

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

  right_speed = ALPHA * speed_e0 + (1.0f - ALPHA) * last_speed_e0;
  left_speed  = ALPHA * speed_e1 + (1.0f - ALPHA) * last_speed_e1;

  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
}

// Convert raw discharge time into a positive signal.
// More IR -> smaller raw reading -> bigger signal.
float readingToSignal(float reading, float baseline) {
  if (baseline <= 1.0f) return 0.0f;

  float x = 100.0f * (baseline - reading) / baseline;

  if (x < 0.0f) x = 0.0f;
  if (x > 100.0f) x = 100.0f;

  return x;
}

float computeBearingSignal(float l_sig, float r_sig) {
  float total_now = l_sig + r_sig;
  if (total_now < BEARING_EPS) return 0.0f;
  return (l_sig - r_sig) / total_now;
}

void updateBumpSignals() {
  bump_sensors.readSensorsDigital();

  L = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
  R = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

  float total_temp = L + R;
  total = 0.7f * total + 0.3f * total_temp;

  float bearing_now = computeBearingSignal(L, R);
  bearingSignal = (1.0f - BEARING_FILTER_ALPHA) * bearingSignal + BEARING_FILTER_ALPHA * bearing_now;
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

  baselineTotal = 0.0f;

  for (int i = 0; i < 50; i++) {
    bump_sensors.readSensorsDigital();

    float Lb = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
    float Rb = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

    baselineTotal += (Lb + Rb);
    delay(5);
  }

  baselineTotal /= 50.0f;
}

void startTargetSampling() {
  targetSampleStart = millis();
  targetSampleSum = 0.0f;
  targetBearingSum = 0.0f;
  targetSampleCount = 0;
  minTotalSeen = 100000.0f;
  maxTotalSeen = -100000.0f;
  total = 0.0f;
  bearingSignal = 0.0f;
}

void finishTargetSampling() {
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = targetSampleSum / (float)targetSampleCount;
  targetBearing = targetBearingSum / (float)targetSampleCount;

  beaconThreshold = baselineTotal + 0.4f * (targetTotal - baselineTotal);

  float observedSpread = maxTotalSeen - minTotalSeen;
  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  Serial.print("targetTotal=");
  Serial.println(targetTotal, 3);
  Serial.print("targetBearing=");
  Serial.println(targetBearing, 4);
}

void driveSteered(float driveDemand, float steerDemand) {
  driveDemand = clampFloat(driveDemand, 0.0f, MAX_FWD_SPEED_DEMAND);
  steerDemand = clampFloat(steerDemand, -MAX_STEER_DEMAND, MAX_STEER_DEMAND);

  driveDemand *= FOLLOW_SIGN;
  steerDemand *= FOLLOW_SIGN;

  float leftDemand  = driveDemand - steerDemand;
  float rightDemand = driveDemand + steerDemand;

  float l_pwm_f = left_pid.update(leftDemand, left_speed);
  float r_pwm_f = right_pid.update(rightDemand, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  l_pwm = applyPwmFloor(l_pwm, leftDemand, PWM_FLOOR_FWD);
  r_pwm = applyPwmFloor(r_pwm, rightDemand, PWM_FLOOR_FWD);

  motors.setPWM(l_pwm, r_pwm);
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

  measureBumpBaseline();

  pose.initialise(0.0f, 0.0f, 0.0f);
  pose_ts = millis();
  robotstate = WAITING_FOR_TARGET_BUTTON;
  oled_ts = millis();
  lastLoop = millis();
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long now = millis();

  if (now - speed_est_ts >= SPEED_EST_MS) {
    speedcalc(now - speed_est_ts);
    speed_est_ts = now;
  }

  updateBumpSignals();
  updatePoseIfNeeded();
  logPoseIfNeeded();

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
        lastButtonState = buttonState;
        return;
      }

      if (robotstate == WAITING_FOR_TARGET_BUTTON) {
        startTargetSampling();
        robotstate = SAMPLING_TARGET;
      }
    }
  }

  if (robotstate == SAMPLING_TARGET) {
    stopRobot();

    if ((millis() - targetSampleStart) > TARGET_SETTLE_MS && total > beaconThreshold) {
      targetSampleSum += total;
      targetBearingSum += bearingSignal;
      targetSampleCount++;

      if (total < minTotalSeen) minTotalSeen = total;
      if (total > maxTotalSeen) maxTotalSeen = total;
    }

    if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
      if (targetSampleCount < 20) {
        stopRobot();
        robotstate = WAITING_FOR_TARGET_BUTTON;
        return;
      }
      finishTargetSampling();
      robotstate = WAITING_FOR_BEACON_OFF;
    }
  }
  else if (robotstate == WAITING_FOR_BEACON_OFF) {
    stopRobot();

    if (total < beaconThreshold) {
      total = 0.0f;
      bearingSignal = 0.0f;
      robotstate = WAITING_FOR_BEACON_ON;
    }
  }
  else if (robotstate == WAITING_FOR_BEACON_ON) {
    stopRobot();

    if (total > beaconThreshold) {
      left_pid.reset();
      right_pid.reset();
      smoothedDrive = 0.0f;
      beaconLostStart = 0;

      logIndex = 0;
      loggingActive = true;
      logReadyToDump = false;
      lastLogTime = millis() - LOG_MS;

      robotstate = FOLLOWING;
    }
  }
  else if (robotstate == FOLLOWING) {
    if (total < beaconThreshold) {
      if (beaconLostStart == 0) beaconLostStart = millis();

      if (millis() - beaconLostStart > LOST_TIMEOUT_MS) {
        stopRobot();
        loggingActive = false;
        logReadyToDump = true;
        robotstate = WAITING_FOR_BEACON_ON;
      } else {
        driveSteered(0.0f, 0.0f);
      }
    }
    else {
      beaconLostStart = 0;

      // ---------------- Longitudinal control ----------------
      float totalError = targetTotal - total;
      float desiredDrive = 0.0f;

      if (totalError > TARGET_DEADBAND) {
        desiredDrive = SPEED_GAIN * (totalError - TARGET_DEADBAND);
      } else {
        desiredDrive = 0.0f;
      }

      desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);
      smoothedDrive = 0.85f * smoothedDrive + 0.15f * desiredDrive;

      // ---------------- Lateral control from sampled bearing ----------------
      float desiredSteer = 0.0f;

      if (total > STEER_ENABLE_TOTAL_FRAC * targetTotal) {
        // flipped sign, since that was the version that behaved better for you
        float steerError = bearingSignal - targetBearing;

        if (fabs(steerError) < BEARING_DEADBAND) {
          steerError = 0.0f;
        }

        float steerScale = total / targetTotal;
        steerScale = clampFloat(steerScale, 0.0f, 1.0f);

        desiredSteer = STEER_GAIN * steerError * steerScale;
      }

      float steerLimit = 0.25f * smoothedDrive + 0.01f;
      if (steerLimit > MAX_STEER_DEMAND) steerLimit = MAX_STEER_DEMAND;

      desiredSteer = clampFloat(desiredSteer, -steerLimit, steerLimit);

      driveSteered(smoothedDrive, desiredSteer);
    }
  }

  lastButtonState = buttonState;

  if (USE_OLED && millis() - oled_ts >= OLED_UPDATE_MS) {
    oled_ts = millis();
    drawFollowerOLED();
  }
}
