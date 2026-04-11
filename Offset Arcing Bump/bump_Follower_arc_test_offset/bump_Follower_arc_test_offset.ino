//==========================================================================//
// FOLLOWER: USE BLUE TACK SCREEN ROBOT - no-spin baseline calibration      //
//==========================================================================//

#include "Motors.h"
#include "BumpSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "Kinematics.h"
#include <math.h>

Motors_c motors;
BumpSensors_c bump_sensors;
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
//                        4. CAL VARIABLES - SENSOR & THRESHOLDS            //
//==========================================================================//

// baseline - beacon OFF, measured oncee at power on
float bump_baseline[BUMP_NUM_SENSORS];
float baselineTotal = 0.0f;
float beaconThreshold = 0.0f;

float total = 0;
float prevTotal = 0;
float L = 0;
float R = 0;
float C = 0;
float smoothedLR = 0.0f;
//==========================================================================//
//                        5. TARGET SAMPLING VARIABLES                      //
//==========================================================================//

const uint32_t TARGET_SETTLE_MS = 300;
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
const float SPEED_GAIN = 0.03f;
const float TARGET_DEADBAND = 0.5f;

// steering
const float STEER_GAIN = 0.0050f;
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
const uint32_t STOP_SETTLE_MS = 500;
uint32_t belowDriveSince = 0;
bool hasStartedFollowingMotion = false;
const float FOLLOW_START_THRESHOLD = 0.03f;

// no-turn startup window
const uint32_t NO_TURN_AFTER_START_MS = 1000;
uint32_t followMotionStartTime = 0;

// Beacon loss dtection from ML's offset based code
const uint32_t LOST_TIMEOUT_MS = 200;
uint32_t beaconLostStart = 0; 

//==========================================================================//
//                        8. FSM SETTING                                    //
//==========================================================================//

enum RobotState {
  WAITING_FOR_TARGET_BUTTON,
  SAMPLING_TARGET,
  WAITING_FOR_BEACON_OFF,
  WAITING_FOR_BEACON_ON,
  FOLLOWING,
  STOPPED
};

RobotState robotState = WAITING_FOR_TARGET_BUTTON;

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
bool lastButtonState = HIGH;

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

// 1. ----------------------------------------------------------------------//
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// 2. -----------------------------------------------------------------------//
int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// 3. -----------------------------------------------------------------------//
int applyPwmFloor(int pwm, float demand_speed, int floorVal) {
  if (fabs(demand_speed) < 1e-6f) return 0;
  if (abs(pwm) < floorVal) {
    pwm = (demand_speed > 0.0f) ? floorVal : -floorVal;
  }
  return pwm;
}

// 4. -----------------------------------------------------------------------//
float angleDiff(float target, float source) {
  return atan2f(sinf(target - source), cosf(target - source));
}

// 5. -----------------------------------------------------------------------//
void setBeep(uint32_t durationMs) {
  analogWrite(BUZZER_PIN, BUZZER_PWM);
  beepStopTime = millis() + durationMs;
}

//6. -----------------------------------------------------------------------//
void updateBeep() {
  if (beepStopTime != 0 && millis() >= beepStopTime) {
    analogWrite(BUZZER_PIN, 0);
    beepStopTime = 0;
  }
}

// 7. -----------------------------------------------------------------------//
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

//==========================================================================//
//                        LOGGER FUNCTIONS                                  //
//==========================================================================//
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
// 1. -----------------------------------------------------------------------//
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

// 2. -----------------------------------------------------------------------//
void updateWheelSpeedsIfNeeded()
{
  unsigned long elapsed_time = millis() - speed_est_ts;
  if (elapsed_time >= SPEED_EST_MS) {
    speedcalc(elapsed_time);
    speed_est_ts = millis();
  }
}

// 3. -----------------------------------------------------------------------//
void updatePoseIfNeeded()
{
  unsigned long now = millis();
  if (now - pose_ts >= POSE_MS) {
    pose_ts = now;
    pose.update();
  }
}

// 4. -----------------------------------------------------------------------//
// Smaller discharge time = stronger IR, so normalization is inverted.
float bumpReadingToPercent(float raw, float minv, float maxv)
{
  float range = maxv - minv;
  if (range < 1.0f) range = 1.0f;

  float norm = (maxv - raw) / range;
  norm = clampFloat(norm, 0.0f, 1.0f);
  return norm * 100.0f;
}

// 5. -----------------------------------------------------------------------//
// NEW ADDITION 
float readingToSignal(float reading, float baseline) {
  if (baseline <= 1.0f) return 0.0f;
  float x = 100.0 * (baseline - reading) / baseline;
  if (x < 0.0f) x = 0.0f;
  if (x > 100.0f) x = 100.0f;
  return x;
}

// 6. -----------------------------------------------------------------------//
void updateSensors() {
  bump_sensors.readSensorsDigital();

  L = readingToSignal(bump_sensors.readings[0], bump_baseline[0]);
  R = readingToSignal(bump_sensors.readings[1], bump_baseline[1]);

  prevTotal = total;
  total = L + R;  
}


//==========================================================================//
//                        CALIBRATION                                       //
//==========================================================================//

void measureBumpBaseline(){
  const int N = 80;

  for (int s = 0; s < BUMP_NUM_SENSORS; s++) bump_baseline[s] = 0.0f;

  for (int i = 0; i < N; i++){
    bump_sensors.readSensorsDigital();
    for (int s = 0; s < BUMP_NUM_SENSORS; s++) { 
      bump_baseline[s] += bump_sensors.readings[s];
    }
    delay(5);
  }

  for (int s = 0; s < BUMP_NUM_SENSORS; s++) bump_baseline[s] /= N;

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
  total = 0.0f;
  setBeep(SAMPLE_BEEP_MS);
}

void finishTargetSampling()
{
  if (targetSampleCount < 1) targetSampleCount = 1;

  targetTotal = targetSampleSum / (float)targetSampleCount;
  targetLR = lrSampleSum / (float)targetSampleCount;

  beaconThreshold = baselineTotal + 0.f * (targetTotal - baselineTotal);

  float observedSpread = maxTotalSeen - minTotalSeen;
  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  Serial.println("target sampling done");
  Serial.print("targetTotal,"); Serial.println(targetTotal);
  Serial.print("lowerThreshold,"); Serial.println(lowerThreshold);
  Serial.print("upperThreshold,"); Serial.println(upperThreshold);
  Serial.print("beaconThreshold,"); Serial.println(beaconThreshold);  
}
//==========================================================================//
//                        STATE FUNCTIONS                                   //
//==========================================================================//
// 1. -----------------------------------------------------------------------//
void stateWaitingForButton(){
  motors.setPWM(0,0);
}

void stateSampling() {
  motors.setPWM(0,0);

  if ((millis() - targetSampleStart) > TARGET_SETTLE_MS && total > beaconThreshold){
    targetSampleSum += total;
    lrSampleSum += (L - R);
    targetSampleCount++;

    if (total < minTotalSeen) minTotalSeen = total;
    if (total > maxTotalSeen) maxTotalSeen = total;
  }
  
  if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
    if (targetSampleCount < 20){
      robotState = WAITING_FOR_TARGET_BUTTON;
      return;
    }
    finishTargetSampling();
    robotState = WAITING_FOR_BEACON_OFF;   
  }
}

void stateWaitingForBeaconOff(){
  motors.setPWM(0,0);
  if (total < beaconThreshold){
    total = 0.0f;
    smoothedLR = 0.0f;
    robotState = WAITING_FOR_BEACON_ON;    
  }
}

void stateWaitingForBeaconOn(){
  motors.setPWM(0,0);
  if (total > beaconThreshold){
    left_pid.reset();
    right_pid.reset();
    smoothedDrive = 0.0f;
    smoothedLR = (L-R) - targetLR;
    belowDriveSince = 0;
    hasStartedFollowingMotion = false;
    followMotionStartTime = 0;
    beaconLostStart = 0; 

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

  // beacon loss detecttion
  if (total < beaconThreshold){
    if (beaconLostStart == 0) beaconLostStart = millis();

    if (millis() - beaconLostStart > LOST_TIMEOUT_MS){
      stopRobot();
      robotState = STOPPED;
      setBeep(STOPPED_BEEP_MS);
      return;
    }
    motors.setPWM(0,0);
    return;
  }
  beaconLostStart = 0;
  
  float error = (targetTotal - TARGET_DEADBAND) - total;
  float desiredDrive = 0.0f;

  if (total < targetTotal - TARGET_DEADBAND) {
    desiredDrive = SPEED_GAIN * error;
  }
  desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);

  smoothedDrive = 0.3f * desiredDrive + 0.7f * smoothedDrive;
   
  if (smoothedDrive > FOLLOW_START_THRESHOLD) {
    if (!hasStartedFollowingMotion) {
      hasStartedFollowingMotion = true;
      followMotionStartTime = millis();
    }
  }

  if (smoothedDrive < 0.05f) {
    if (hasStartedFollowingMotion) {
      if (belowDriveSince == 0) {
        belowDriveSince = millis();
      }

      motors.setPWM(0, 0);

      if (millis() - belowDriveSince >= STOP_SETTLE_MS) {
        stopRobot();
        robotState = STOPPED;
        setBeep(STOPPED_BEEP_MS);
      }
    } else {
      belowDriveSince = 0;
      motors.setPWM(0, 0);
    }
    return;
  } else {
    if (smoothedDrive > 0.12f){
    belowDriveSince = 0;
    }
  }

  float rawLRDiff = (L - R) - targetLR;
  float steer = 0.0f;

  bool steeringEnabled = true;
  if (hasStartedFollowingMotion) {
    if (millis() - followMotionStartTime < NO_TURN_AFTER_START_MS) {
      steeringEnabled = false;
    }
  }

  smoothedLR = 0.5f * rawLRDiff + 0.5f * smoothedLR;

  if (steeringEnabled) {
    steer = clampFloat(-STEER_GAIN * smoothedLR, -MAX_STEER, MAX_STEER);
  } else {
    steer = 0.0f;
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
//                        SETUP                                             //
//==========================================================================//

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 0);

  motors.initialise();
  bump_sensors.initialiseForDigital();
  bump_sensors.timeout_us = 3000;
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

  measureBumpBaseline();

  robotState = WAITING_FOR_TARGET_BUTTON;
  smoothedDrive = 0.0f;
  belowDriveSince = 0;
  hasStartedFollowingMotion = false;
  followMotionStartTime = 0;
  
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

  if (robotState == SAMPLING_TARGET || 
      robotState == WAITING_FOR_BEACON_OFF ||
      robotState == WAITING_FOR_BEACON_ON ||
      robotState == FOLLOWING) {
    updateSensors();
  }
  
  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (lastButtonState ==  HIGH && buttonState == LOW){
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW){
      if(logReadyToDump){
        Serial.println(F("i_mm, x_mm, y_mm, ts"));
        for(int i = 0; i < logIndex; i++){
          float x = logBuffer[i].x_centi/ 100.0f;
          float y = logBuffer[i].y_centi/ 100.0f - 300.0f;
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
      if (robotState == WAITING_FOR_TARGET_BUTTON){
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
    case WAITING_FOR_BEACON_OFF:
      stateWaitingForBeaconOff();
      break;
     case WAITING_FOR_BEACON_ON:
      stateWaitingForBeaconOn();
      break; 
    case FOLLOWING:
      stateFollowing();
      break;
    case STOPPED:
      stateStopped();
      break;
  }

  logData();
  

//  Serial.print(L);
//  Serial.print(",");
//  Serial.print(R);
//  Serial.print(",");
//  Serial.println(total);
}
