#include "Motors.h"
#include "LineSensors.h"
#include "PID.h"
#include "Encoders.h"

Motors_c motors;
PID_c left_pid;
PID_c right_pid;

// button pin
const int BUTTON_A_PIN = 14;

// motion
const uint32_t REVERSE_TIME_MS = 5000;

// speed demand for straight reverse
const float REVERSE_SPEED_DEMAND = -0.35f;

// control / estimation timing
const uint32_t SPEED_EST_MS = 10;
const uint32_t CTRL_MS = 20;
const float ALPHA = 0.2f;

// pwm limits
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR = 18;

// pid gains
const float DRIVE_KP = 15.0f;
const float DRIVE_KI = 0.1f;
const float DRIVE_KD = 0.0f;

// state
bool reversing = false;
bool lastButtonState = HIGH;
uint32_t reverseStartTime = 0;

// timing
uint32_t lastCtrlTime = 0;
uint32_t lastSpeedTime = 0;

// encoder history
long last_e0 = 0;
long last_e1 = 0;

// measured wheel speeds
float right_speed = 0.0f;
float left_speed = 0.0f;
float last_speed_e0 = 0.0f;
float last_speed_e1 = 0.0f;

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
}

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

void updateReverseControlIfNeeded() {
  uint32_t now = millis();
  if (now - lastCtrlTime < CTRL_MS) return;
  lastCtrlTime = now;

  float l_pwm_f = left_pid.update(REVERSE_SPEED_DEMAND, left_speed);
  float r_pwm_f = right_pid.update(REVERSE_SPEED_DEMAND, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  l_pwm = applyPwmFloor(l_pwm, REVERSE_SPEED_DEMAND, PWM_FLOOR);
  r_pwm = applyPwmFloor(r_pwm, REVERSE_SPEED_DEMAND, PWM_FLOOR);

  motors.setPWM(l_pwm, r_pwm);
}

void setup()
{
  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  // turn IR emitters on immediately
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);

  // button input with pull-up
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  lastSpeedTime = millis();
  lastCtrlTime = millis();

  // start still
  motors.setPWM(0, 0);
}

void loop()
{
  bool buttonState = digitalRead(BUTTON_A_PIN);

  // detect a new button press: HIGH -> LOW
  if (!reversing && lastButtonState == HIGH && buttonState == LOW) {
    reversing = true;
    reverseStartTime = millis();
    left_pid.reset();
    right_pid.reset();
  }

  updateWheelSpeedsIfNeeded();

  if (reversing) {
    updateReverseControlIfNeeded();

    // stop after 5 seconds
    if (millis() - reverseStartTime >= REVERSE_TIME_MS) {
      reversing = false;
      stopRobot();
    }
  } else {
    motors.setPWM(0, 0);
  }

  lastButtonState = buttonState;
}
