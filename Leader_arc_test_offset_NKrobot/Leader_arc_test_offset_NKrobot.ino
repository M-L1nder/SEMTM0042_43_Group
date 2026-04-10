#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"
#include "Encoders.h"
#include "Kinematics.h"
#include <math.h>

Motors_c motors;
PID_c left_pid;
PID_c right_pid;
PID_c heading_pid;
Kinematics_c pose;

// button pin
const int BUTTON_A_PIN = 14;

// target coordinates
float TARGET_X = -100.0f;
float TARGET_Y = -00.0f;
bool travelling = false;

const float DIST_TOL = 5.0;

// timing variables
const uint32_t SPEED_EST_MS = 10;
const uint32_t CTRL_MS = 20;
const uint32_t POSE_MS = 20;

uint32_t lastCtrlTime = 0;
uint32_t lastSpeedTime = 0;
uint32_t lastPoseTime = 0;

// pwm limits
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR = 18;

// pid settings & steering
const float DRIVE_KP = 60.0f;
const float DRIVE_KI = 0.02f;
const float DRIVE_KD = 0.001f;

const float HEAD_KP = 2.0f;
const float HEAD_KI = 0.0f;
const float HEAD_KD = 0.0f;

const float REVERSE_SPEED = 0.45f;
const float MAX_STEER = 0.04f;

// state variables
bool lastButtonState = HIGH;

// wheel speed estimation variables
long last_e0 = 0;
long last_e1 = 0;

float right_speed = 0.0f;
float left_speed = 0.0f;
float last_speed_e0 = 0.0f;
float last_speed_e1 = 0.0f;

const float ALPHA = 0.2f;

// logging
const uint32_t LOG_MS = 50;
const int LOG_SIZE = 200;
const uint32_t STOP_SETTLE_MS = 1000;
const float MOTION_START_SPEED = 0.03f;
const float STOP_SPEED_THRESHOLD = 0.01f;

struct LogEntry {
  uint32_t t_ms;
  int16_t x_centi; // pose.x * 100
  int16_t y_centi; // pose.y * 100
};

LogEntry logBuffer[LOG_SIZE];
int logIndex = 0;
uint32_t lastLogTime = 0;
bool loggingActive = false;
bool logReadyToDump = false;
bool hasStartedMotion = false;
uint32_t belowSpeedSince = 0;

// FSM states
enum RobotState {
  WAITING,
  REVERSING,
  DONE,
  STOPPED
};

RobotState state = WAITING;

// helper functions
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
  if (demand_speed == 0.0f) return 0;
  if (abs(pwm) < floorVal) {
    pwm = (demand_speed > 0.0f) ? floorVal : -floorVal;
  }
  return pwm;
}

void stopRobot() {
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
  heading_pid.reset();

  if (loggingActive) {
    loggingActive = false;
    logReadyToDump = true;
  }
}

float angleDiff(float target) {
  return atan2f(sinf(target - pose.theta), cosf(target - pose.theta));
}

void startLogging() {
  logIndex = 0;
  lastLogTime = millis() - LOG_MS;
  loggingActive = true;
  logReadyToDump = false;
  hasStartedMotion = false;
  belowSpeedSince = 0;
}

void logData() {
  if (!loggingActive) return;

  uint32_t now = millis();
  if (now - lastLogTime < LOG_MS) return;
  lastLogTime = now;

  if (logIndex >= LOG_SIZE) return;

  logBuffer[logIndex].x_centi = (int16_t)(pose.x * 100.0f);
  logBuffer[logIndex].y_centi = (int16_t)(pose.y * 100.0f);
  logBuffer[logIndex].t_ms = now;
  logIndex++;
}

void logDump() {
  bool buttonState = digitalRead(BUTTON_A_PIN);
  static bool lastBtn = HIGH;

  if (lastBtn == HIGH && buttonState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW && logReadyToDump) {
      Serial.println(F("i,x_mm,y_mm,ts"));
      for (int i = 0; i < logIndex; i++) {
        float x = logBuffer[i].x_centi / 100.0f;
        float y = logBuffer[i].y_centi / 100.0f;
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

// wheel speed estimation
void updateWheelSpeedsIfNeeded() {
  uint32_t now = millis();
  uint32_t elapsed = now - lastSpeedTime;

  if (elapsed < SPEED_EST_MS) return;

  lastSpeedTime = now;

  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;

  last_e0 = count_e0;
  last_e1 = count_e1;

  float speed_e0 = (float)delta_e0 / (float)elapsed;
  float speed_e1 = (float)delta_e1 / (float)elapsed;

  right_speed = ALPHA * speed_e0 + (1.0f - ALPHA) * last_speed_e0;
  left_speed  = ALPHA * speed_e1 + (1.0f - ALPHA) * last_speed_e1;

  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
}

// pose update
void updatePoseifNeeded() {
  uint32_t now = millis();
  if (now - lastPoseTime < POSE_MS) return;
  lastPoseTime = now;

  pose.update();
}

// reverse pid control
void setTravel(float x, float y) {
  TARGET_X = x;
  TARGET_Y = y;
  travelling = true;
  left_pid.reset();
  right_pid.reset();
  heading_pid.reset();
  startLogging();
}

bool checkTravelReverse() {
  if (!travelling) {
    return true;
  }

  float dx = TARGET_X - pose.x;
  float dy = TARGET_Y - pose.y;
  float dist = sqrtf(dx * dx + dy * dy);

  // Stop if within 5 mm of target, OR x < -305, OR y < -105
  if (dist <= DIST_TOL || pose.x < -305.0f || pose.y < -105.0f) {
    motors.setPWM(0, 0);
    travelling = false;
    left_pid.reset();
    right_pid.reset();
    heading_pid.reset();
    return true;
  }

  float desired_heading = atan2f(dy, dx);
  desired_heading += PI;

  float heading_error = angleDiff(desired_heading);
  float rev = REVERSE_SPEED;
  float head_corr = 2.0f * heading_pid.update(0.0f, heading_error);
  float steer_limit = 0.07f * rev;
  head_corr = clampFloat(head_corr, -steer_limit, steer_limit);

  if (dist < 50.0f) {
    rev = 0.3f;
  }

  float left_pwm  = left_pid.update(-rev + head_corr, left_speed);
  float right_pwm = right_pid.update(-rev - head_corr, right_speed);

  motors.setPWM((int)left_pwm, (int)right_pwm);
  return false;
}

// state functions
void stateWaiting() {
  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (lastButtonState == HIGH && buttonState == LOW) {
    setTravel(TARGET_X, TARGET_Y);
    state = REVERSING;
  }
  lastButtonState = buttonState;
}

void stateReverse() {
  if (fabs(left_speed) > MOTION_START_SPEED || fabs(right_speed) > MOTION_START_SPEED) {
    hasStartedMotion = true;
  }

  if (checkTravelReverse()) {
    stopRobot();
    belowSpeedSince = millis();
    state = DONE;
  }
}

void stateDone() {
  motors.setPWM(0, 0);

  float maxAbsSpeed = fabs(left_speed);
  if (fabs(right_speed) > maxAbsSpeed) maxAbsSpeed = fabs(right_speed);

  if (maxAbsSpeed < STOP_SPEED_THRESHOLD) {
    if (belowSpeedSince == 0) {
      belowSpeedSince = millis();
    }

    if (hasStartedMotion && (millis() - belowSpeedSince >= STOP_SETTLE_MS)) {
      state = STOPPED;
    }
  } else {
    belowSpeedSince = 0;
  }
}

void stateStopped() {
  motors.setPWM(0, 0);
}

// setup function
void setup() {
  Serial.begin(9600);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, HIGH);

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  pose.initialise(0, 0, 0);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  heading_pid.initialise(HEAD_KP, HEAD_KI, HEAD_KD);
  left_pid.reset();
  right_pid.reset();
  heading_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  lastSpeedTime = millis();
  lastCtrlTime = millis();
  lastPoseTime = millis();

  motors.setPWM(0, 0);
  state = WAITING;
}

// loop
void loop() {
  updateWheelSpeedsIfNeeded();
  updatePoseifNeeded();

  switch (state) {
    case WAITING:
      stateWaiting();
      break;
    case REVERSING:
      stateReverse();
      break;
    case DONE:
      stateDone();
      break;
    case STOPPED:
      stateStopped();
      break;
  }

  logData();
  logDump();
}
