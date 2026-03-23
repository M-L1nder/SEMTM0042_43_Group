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
float TARGET_X = -300.0f; 
float TARGET_Y = -100.0f; 
bool travelling = false;

const float DIST_TOL = 5.0; 

// timing variables
const uint32_t SPEED_EST_MS = 10;
const uint32_t CTRL_MS = 20;
const uint32_t POSE_MS = 20; 
const uint32_t PRINT_MS = 100;

uint32_t lastCtrlTime = 0;
uint32_t lastSpeedTime = 0;
uint32_t lastPoseTime = 0;
uint32_t lastPrintTime = 0;

// pwm limits
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR = 18;

// pid settings & steering
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

const float HEAD_KP = 2.0f;
const float HEAD_KI = 0.0f;
const float HEAD_KD = 0.0f;

const float REVERSE_SPEED = 0.35f;
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

// FSM states
enum RobotState {
  WAITING,
  REVERSING,
  DONE
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
}

float angleDiff(float target){
  return atan2f(sinf(target - pose.theta), cosf(target - pose.theta));
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
void updatePoseifNeeded(){
  uint32_t now = millis();
  if (now - lastPoseTime < POSE_MS) return; 
  lastPoseTime = now;

  pose.update();
}

// reverse pid control
void setTravel(float x, float y){
  TARGET_X = x; 
  TARGET_Y = y; 
  travelling = true;
  left_pid.reset();
  right_pid.reset();
  heading_pid.reset();
}

bool checkTravelReverse() {
  if (!travelling){
    return true;
  }

  float dx = TARGET_X - pose.x;
  float dy = TARGET_Y - pose.y;
  float dist = sqrtf(dx * dx + dy * dy); 

  // Stop if within 5 mm of target, OR x < -305, OR y < -105
  if (dist <= DIST_TOL || pose.x < -305.0f || pose.y < -105.0f){
    motors.setPWM(0,0);
    travelling = false;
    left_pid.reset();
    right_pid.reset();
    heading_pid.reset();
    return true; 
  }

  float desired_heading = atan2f(dy,dx);
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
void stateWaiting(){ 
  bool buttonState = digitalRead(BUTTON_A_PIN);

  if (lastButtonState == HIGH && buttonState == LOW) { 
    setTravel(TARGET_X, TARGET_Y);    
    state = REVERSING;
  }
  lastButtonState = buttonState;
}

void stateReverse(){ 
  if  (checkTravelReverse()){  
    stopRobot();
    state = DONE;
  }
}

void stateDone(){
  motors.setPWM(0, 0);
}

// setup function
void setup()
{
  Serial.begin(9600);
  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  pose.initialise(0.0f, 0.0f, 0.0f);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  heading_pid.initialise(HEAD_KP, HEAD_KI, HEAD_KD);
  left_pid.reset();
  right_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  lastSpeedTime = millis();
  lastCtrlTime = millis();
  lastPoseTime = millis();
  lastPrintTime = millis();

  motors.setPWM(0, 0);
  state = WAITING;
}

// loop
void loop()
{
  updateWheelSpeedsIfNeeded();
  updatePoseifNeeded();

  switch (state){
    case WAITING:
      stateWaiting();
      break;
    case REVERSING:
      stateReverse();
      break;
    case DONE: 
      stateDone();
      break;
  }

  uint32_t now = millis();
  if (now - lastPrintTime >= PRINT_MS) {
    lastPrintTime = now;
    Serial.print(pose.x);
    Serial.print(",");
    Serial.print(pose.y);
    Serial.print(",");
    Serial.println(pose.theta);
  }
}
