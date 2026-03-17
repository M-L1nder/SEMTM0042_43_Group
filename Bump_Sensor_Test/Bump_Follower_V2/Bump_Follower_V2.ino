#include "Motors.h"
#include "BumpSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "oled.h"
OLED_c display(1, 30, 0, 17, 13);   // clk, mosi, rst, dc, cs

unsigned long oled_ts = 0;
#define OLED_UPDATE_MS 200   // fairly responsive without flicker


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
float lowerThreshold = 0.0f;
float upperThreshold = 0.0f;

float targetSampleSum = 0.0f;
int targetSampleCount = 0;
float minTotalSeen = 100000.0f;
float maxTotalSeen = -100000.0f;

// -------------------- TUNING --------------------
// target capture time
const uint32_t TARGET_SAMPLE_TIME_MS = 2000;

// after target capture, wait for beacon to disappear, then reappear for run start
const float BEACON_PRESENT_TOTAL = 15.0f;   // tune from serial logs
const uint32_t LOST_TIMEOUT_MS = 200;

// drive control
const float MAX_FWD_SPEED_DEMAND = 0.45f;  // should be a bit above leader speed magnitude
const float SPEED_GAIN = 0.015f;           // converts signal deficit -> speed demand
const float TARGET_DEADBAND = 3.0f;        // band around target before moving

// threshold band built from observed target variation
const float BAND_MARGIN_FRACTION = 0.20f;
const float MIN_BAND_MARGIN = 5.0f;

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
  display.print("/");
  display.print(targetTotal, 1);
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

void updateWheelSpeedsIfNeeded() {
  unsigned long elapsed_time = millis() - speed_est_ts;
  if (elapsed_time >= SPEED_EST_MS) {
    speedcalc(elapsed_time);
    speed_est_ts = millis();
  }
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

void updateBumpSignals() {
  bump_sensors.readSensorsDigital();

  L = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
  R = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

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
  targetSampleSum = 0.0f;
  targetSampleCount = 0;
  minTotalSeen = 100000.0f;
  maxTotalSeen = -100000.0f;
}

void finishTargetSampling() {
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = targetSampleSum / (float)targetSampleCount;

  float observedSpread = maxTotalSeen - minTotalSeen;
  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

//  Serial.println("target sampling done");
//  Serial.print("targetTotal,"); Serial.println(targetTotal);
//  Serial.print("lowerThreshold,"); Serial.println(lowerThreshold);
//  Serial.print("upperThreshold,"); Serial.println(upperThreshold);
}

void driveAtDemand(float demandSpeed) {
  demandSpeed = clampFloat(demandSpeed, 0.0f, MAX_FWD_SPEED_DEMAND);
  demandSpeed *= FOLLOW_SIGN;

  float l_pwm_f = left_pid.update(demandSpeed, left_speed);
  float r_pwm_f = right_pid.update(demandSpeed, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  l_pwm = applyPwmFloor(l_pwm, demandSpeed, PWM_FLOOR_FWD);
  r_pwm = applyPwmFloor(r_pwm, demandSpeed, PWM_FLOOR_FWD);

  motors.setPWM(l_pwm, r_pwm);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  motors.initialise();
  bump_sensors.initialiseForDigital();
  setupEncoder0();
  setupEncoder1();

  // Faster timeout helps responsiveness
  bump_sensors.timeout_us = 2500;

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  speed_est_ts = millis();

  // Baseline must be measured with beacon OFF
  Serial.println("Measure baseline: keep leader beacon OFF");
  measureBumpBaseline();

  Serial.println("Place leader at desired distance, beacon ON, then press follower button A");
 
  robotstate = WAITING_FOR_TARGET_BUTTON;
  drawFollowerOLED();
  oled_ts = millis();
  lastLoop = millis();
}

// -------------------- LOOP --------------------
void loop() {
  updateWheelSpeedsIfNeeded();

  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  // Update bump signals in all states after baseline
  updateBumpSignals();

  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (robotstate == WAITING_FOR_TARGET_BUTTON) {
    stopRobot();

    // press follower button to begin target capture
    if (lastButtonState == HIGH && buttonState == LOW) {
      startTargetSampling();
      robotstate = SAMPLING_TARGET;
    }
  }
  else if (robotstate == SAMPLING_TARGET) {
    stopRobot();

    // accumulate only while beacon is actually present
    if (total > BEACON_PRESENT_TOTAL) {
      targetSampleSum += total;
      targetSampleCount++;

      if (total < minTotalSeen) minTotalSeen = total;
      if (total > maxTotalSeen) maxTotalSeen = total;
    }

    if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
      finishTargetSampling();
      robotstate = WAITING_FOR_BEACON_OFF;
    }
  }
  else if (robotstate == WAITING_FOR_BEACON_OFF) {
    stopRobot();

    // after target capture, wait until beacon is turned off
    if (total < BEACON_PRESENT_TOTAL) {
      robotstate = WAITING_FOR_BEACON_ON;
    }
  }
  else if (robotstate == WAITING_FOR_BEACON_ON) {
    stopRobot();

    // start following only when beacon reappears for the real run
    if (total > BEACON_PRESENT_TOTAL) {
      left_pid.reset();
      right_pid.reset();
      smoothedDrive = 0.0f;
      beaconLostStart = 0;
      robotstate = FOLLOWING;
    }
  }
  else if (robotstate == FOLLOWING) {
    if (total < BEACON_PRESENT_TOTAL) {
      // beacon lost: give it a short grace period, then stop
      if (beaconLostStart == 0) beaconLostStart = millis();

      if (millis() - beaconLostStart > LOST_TIMEOUT_MS) {
        stopRobot();
        robotstate = WAITING_FOR_BEACON_ON;
      } else {
        driveAtDemand(0.0f);
      }
    }
    else {
      beaconLostStart = 0;

      float desiredDrive = 0.0f;

      // For bump signals: smaller total = farther away / weaker beacon
      // So drive forward only if we are BELOW the target band.
      if (total < targetTotal - TARGET_DEADBAND) {
        desiredDrive = SPEED_GAIN * ((targetTotal - TARGET_DEADBAND) - total);
      } else {
        desiredDrive = 0.0f;
      }

      desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);

      // no smoothing, same as your good line-sensor version
      smoothedDrive = desiredDrive;

      driveAtDemand(smoothedDrive);
    }
  }

  lastButtonState = buttonState;

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
  Serial.print(L);
  Serial.print(",");
  Serial.print(R);
  Serial.print(",");
  Serial.println((int)robotstate);
  if (millis() - oled_ts >= OLED_UPDATE_MS) {
  oled_ts = millis();
  drawFollowerOLED();
}
}
