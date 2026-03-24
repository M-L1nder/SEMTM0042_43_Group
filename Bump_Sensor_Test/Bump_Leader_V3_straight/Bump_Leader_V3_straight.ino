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

// 14. Connect the cord and press A on leader to download the data
//
// Notes:
// - Baseline measurement must always be done with leader beacon OFF.
// - Target capture must always be done with leader beacon ON and stationary. - computes beacon present threshold from this
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
#include "Kinematics.h"

Motors_c motors;
PID_c left_pid;
PID_c right_pid;
PID_c heading_pid;
Kinematics_c pose;
// -------------------- Logging --------------------
const uint32_t LOG_MS = 50;
const int LOG_SIZE = 100;

struct LogEntry {
  uint16_t x_centi;   // stores (x + 400) * 100
  uint16_t y_centi;   // stores (y + 300) * 100
  //  int16_t theta_mrad;
  uint32_t t_ms;
};

LogEntry logBuffer[LOG_SIZE];
int logIndex = 0;
uint32_t lastLogTime = 0;
bool loggingActive = false;
bool logReadyToDump = false;

// -------------------- BUTTONS --------------------
// Adjust these if your board uses different pins
const int BUTTON_A_PIN = 14;   // toggle stationary beacon mode
const int BUTTON_B_PIN = 30;   // start reverse run with 800ms delay

// -------------------- MOTION --------------------
const uint32_t REVERSE_TIME_MS = 2000;
const float REVERSE_SPEED_DEMAND = -0.35;

// target coordinates
float TARGET_X = -300.0f;
float TARGET_Y = -200.0f;
bool travelling = false;

const float DIST_TOL = 3.0;

// -------------------- TIMING --------------------
const uint32_t SPEED_EST_MS = 10;
const uint32_t CTRL_MS = 20;
const float ALPHA = 0.2;
const uint32_t PRESTART_BEACON_MS = 800;

const uint32_t POSE_MS = 20;
const uint32_t PRINT_MS = 100;

// -------------------- PWM LIMITS --------------------
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR = 18;

// -------------------- PID GAINS --------------------
const float DRIVE_KP = 15.0;
const float DRIVE_KI = 0.1;
const float DRIVE_KD = 0.0;
const float HEAD_KP = 0.5f;
const float HEAD_KI = 0.0f;
const float HEAD_KD = 0.0f;
const float REVERSE_SPEED = 0.35f;
const float MAX_STEER = 0.04f;
const float KY  = 0.01f;
const float KTH = 0.8f;
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
uint32_t lastPoseTime = 0;
uint32_t lastPrintTime = 0;

// encoder history
long last_e0 = 0;
long last_e1 = 0;

// measured wheel speeds
float right_speed = 0.0;
float left_speed = 0.0;
float last_speed_e0 = 0.0;
float last_speed_e1 = 0.0;

// -------------------- HELPERS --------------------
void logPoseIfNeeded() {
  if (!loggingActive) return;

  uint32_t now = millis();
  if (now - lastLogTime < LOG_MS) return;
  lastLogTime = now;

  if (logIndex >= LOG_SIZE) return;

  logBuffer[logIndex].x_centi = (uint16_t)((pose.x + 400.0f) * 100.0f);
  logBuffer[logIndex].y_centi = (uint16_t)((pose.y + 300.0f) * 100.0f);
  logBuffer[logIndex].t_ms = millis();
  //  logBuffer[logIndex].theta_mrad = (int16_t)(pose.theta * 1000.0f);

  logIndex++;
}

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


// Returns smallest difference between any two angles
// -ve number indicating a clockwise turn, +ve number is anticlockwise turn
float angleDiff(float target, float source) {
  float diff = target - source;
  if (diff > PI)  diff -= 2.0 * PI;
  if (diff <= -PI) diff += 2.0 * PI;
  return diff;
}
//float angleDiff(float target) {
//  return atan2f(sinf(target - pose.theta), cosf(target - pose.theta));
//}

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
  heading_pid.reset();
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

  float heading_error = angleDiff(desired_heading, pose.theta);
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


// -------------------- SETUP --------------------
void setup()
{
  Serial.begin(115200);
  //  Serial.println("robot,time_ms,x_mm,y_mm,theta_rad,state");
  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(12, DRIVE_KI, DRIVE_KD);
  heading_pid.initialise(HEAD_KP, HEAD_KI, HEAD_KD);
  left_pid.reset();
  right_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  lastSpeedTime = millis();
  lastCtrlTime = millis();
  pose.initialise(0.0f, 0.0f, 0.0f);
  lastPoseTime = millis();
  lastPrintTime = millis();
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

      // If log ready → dump instead of toggling beacon
      if (logReadyToDump) {
        Serial.begin(115200);
        delay(1000);

        Serial.println(F("i,x_mm,y_mm,ts"));

        for (int i = 0; i < logIndex; i++) {
          float x = logBuffer[i].x_centi / 100.0f - 400.0f;
          float y = logBuffer[i].y_centi / 100.0f - 300.0f;
          Serial.print(i);
          Serial.print(",");
          Serial.print(x);
          Serial.print(",");
          Serial.print(y);
          Serial.print(",");
          Serial.println(logBuffer[i].t_ms);
          //          Serial.println(logBuffer[i].theta_mrad / 1000.0f);
        }

        return;
      }

      // normal behaviour
      stopRobot();

      if (state == WAITING) {
        state = TARGET_BEACON;
      } else if (state == TARGET_BEACON) {
        state = WAITING;
      }
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

        logIndex = 0;
        loggingActive = true;
        logReadyToDump = false;
        lastLogTime = millis() - LOG_MS;
      }
    }
  }

  updateWheelSpeedsIfNeeded();
  updatePoseifNeeded();
  logPoseIfNeeded();

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
    if  (checkTravelReverse()) {
      loggingActive = false;
      logReadyToDump = true;

      state = WAITING;
      stopRobot();
      beaconOff();
    }
  }
  else if (state == PRESTART) {
    // Beacon ON, but hold still briefly so follower can lock on
    beaconOn();
    motors.setPWM(0, 0);
    //    logIndex = 0;
    //    loggingActive = true;
    //    logReadyToDump = false;
    //    lastLogTime = millis() - LOG_MS;
    if (millis() - prestartStartTime >= PRESTART_BEACON_MS) {
      setTravel(TARGET_X, TARGET_Y);
      left_pid.reset();
      right_pid.reset();
      state = REVERSING;
    }
  }

  lastButtonAState = buttonAState;
  lastButtonBState = buttonBState;
  //  uint32_t now = millis();
  //  if (now - lastPrintTime >= PRINT_MS) {
  //    lastPrintTime = now;
  //    Serial.print("F,");
  //    Serial.print(millis());
  //    Serial.print(pose.x);
  //    Serial.print(",");
  //    Serial.print(pose.y);
  //    Serial.print(",");
  //    Serial.print(pose.theta);
  //    Serial.print(",");
  //    Serial.print(buttonAState);
  //    Serial.print(",");
  //    Serial.print(buttonBState);
  //    Serial.print(",");
  //    Serial.println(state);
  //  }
}
