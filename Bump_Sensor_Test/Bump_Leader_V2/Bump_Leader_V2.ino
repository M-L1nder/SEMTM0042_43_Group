// Blue screen robot was leader, ML robot was follower - it a bit wonky
// Done in daytime with windows uncovered
// 
// Tested Workflow:
// 1. Place the leader at the desired following distance:
//    - leader should be centred in front of the follower
//    - leader should remain stationary
//
// 2. Turn on the LEADER as IDLE:
//    - beacon OFF
//    - motors stopped
//
// 3. Turn on the FOLLOWER :
//    - follower measures bump-sensor background / ambient baseline
//    - leader beacon must stay OFF during this step
//
// 4. Press leader button A:
//    - leader enters TARGET_BEACON mode
//    - beacon turns ON
//    - leader stays still during this 
//
// 5. Press follower button A:
//    - follower starts SAMPLING_TARGET
//    - follower records bump-sensor total signal for the desired spacing
//
// 6. Wait until target sampling finishes:
//    - follower calculates targetTotal and target band
//    - Moves into WAIT_OFF state
//
// 7. Press leader button A again:
//    - leader returns to IDLE
//    - beacon turns OFF
//
// 8. Wait for follower to detect beacon OFF:
//    - follower transitions through WAITING_FOR_BEACON_OFF
//    - then enters WAITING_FOR_BEACON_ON
//
// 9. Press leader button B:
//    - leader enters PRESTART
//    - beacon turns ON
//    - leader stays still briefly to give follower time to lock on
//
// 10. After the prestart delay:
//     - leader enters REVERSING
//     - leader drives backwards for the fixed run time
//
// 11. Follower sees beacon ON again:
//     - follower leaves WAITING_FOR_BEACON_ON
//     - follower enters FOLLOWING
//     - follower drives forward only when total signal is below target band
//
// 12. At the end of the leader run:
//     - leader stops
//     - beacon turns OFF
//
// 13. Follower detects beacon loss:
//     - after a short timeout it stops
//     - follower returns to waiting state
//
// Notes:
// - Baseline measurement must always be done with leader beacon OFF.
// - Target capture must always be done with leader beacon ON and stationary.
// - Straight-line following currently uses total bump IR only, not left-right steering.
// - If the follower does not start, check that it reached WAITING_FOR_BEACON_ON before pressing leader button B.
// - Currently follower outputs Bigger number = more IR  =  Smaller discharge time for ease of use 
// - The follower is sensitive to ambient light so try and not move around, 
// - if the follower T goes above threshold (14 in this current build) as a baseline, it will never go out off WAIT_OFF
// - So press reset and try again 



#include "Motors.h"
#include "LineSensors.h"   // for EMIT_PIN
#include "PID.h"
#include "Encoders.h"

Motors_c motors;
PID_c left_pid;
PID_c right_pid;

// -------------------- BUTTONS --------------------
// Adjust these if your board uses different pins
const int BUTTON_A_PIN = 14;   // toggle stationary beacon mode
const int BUTTON_B_PIN = 30;   // start reverse run with 800ms delay

// -------------------- MOTION --------------------
const uint32_t REVERSE_TIME_MS = 2000;
const float REVERSE_SPEED_DEMAND = -0.35;

// -------------------- TIMING --------------------
const uint32_t SPEED_EST_MS = 10;
const uint32_t CTRL_MS = 20;
const float ALPHA = 0.2;
const uint32_t PRESTART_BEACON_MS = 800;
// -------------------- PWM LIMITS --------------------
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR = 18;

// -------------------- PID GAINS --------------------
const float DRIVE_KP = 15.0;
const float DRIVE_KI = 0.1;
const float DRIVE_KD = 0.0;

// -------------------- STATE --------------------
enum LeaderState {
  WAITING,
  TARGET_BEACON,
  PRESTART,
  REVERSING
};
uint32_t prestartStartTime = 0;
LeaderState state = WAITING;

// button edge tracking
bool lastButtonAState = HIGH;
bool lastButtonBState = HIGH;

// reverse timing
uint32_t reverseStartTime = 0;

// control timing
uint32_t lastCtrlTime = 0;
uint32_t lastSpeedTime = 0;

// encoder history
long last_e0 = 0;
long last_e1 = 0;

// measured wheel speeds
float right_speed = 0.0;
float left_speed = 0.0;
float last_speed_e0 = 0.0;
float last_speed_e1 = 0.0;

// -------------------- HELPERS --------------------
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

void beaconOn() {
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);   // bump IR emitters ON
}

void beaconOff() {
  pinMode(EMIT_PIN, INPUT);      // emitters OFF
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

// -------------------- SETUP --------------------
void setup()
{
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  lastSpeedTime = millis();
  lastCtrlTime = millis();

  // start with beacon OFF and motors OFF
  beaconOff();
  motors.setPWM(0, 0);
}

// -------------------- LOOP --------------------
void loop()
{
  bool buttonAState = digitalRead(BUTTON_A_PIN);
  bool buttonBState = digitalRead(BUTTON_B_PIN);

  // --- Button A toggles stationary beacon mode ---
  if (lastButtonAState == HIGH && buttonAState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW) {
      stopRobot();

      if (state == WAITING) {
        state = TARGET_BEACON;
      } else if (state == TARGET_BEACON) {
        state = WAITING;
      }
      // ignore A while reversing
    }
  }

  // --- Button B starts reverse run from IDLE only ---
  if (lastButtonBState == HIGH && buttonBState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_B_PIN) == LOW) {
      if (state == WAITING) {
        stopRobot();
        state = PRESTART;
        prestartStartTime = millis();
      }
    }
  }

  updateWheelSpeedsIfNeeded();

  if (state == WAITING) {
    beaconOff();
    motors.setPWM(0, 0);
  }
  else if (state == TARGET_BEACON) {
    beaconOn();
    motors.setPWM(0, 0);
  }
  else if (state == REVERSING) {
    beaconOn();
    updateReverseControlIfNeeded();

    if (millis() - reverseStartTime >= REVERSE_TIME_MS) {
      state = WAITING;
      stopRobot();
      beaconOff();
    }
  }
  else if (state == PRESTART) {
    // Beacon ON, but hold still briefly so follower can lock on
    beaconOn();
    motors.setPWM(0, 0);

    if (millis() - prestartStartTime >= PRESTART_BEACON_MS) {
      left_pid.reset();
      right_pid.reset();
      reverseStartTime = millis();
      state = REVERSING;
    }
  }

  lastButtonAState = buttonAState;
  lastButtonBState = buttonBState;

  Serial.print(buttonAState);
  Serial.print(",");
  Serial.print(buttonBState);
  Serial.print(",");
  Serial.println(state);

}
