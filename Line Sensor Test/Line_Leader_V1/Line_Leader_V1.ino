// Leader reverses in a straight line with its
// downward-facing line-sensor IR emitters turned ON so that
// their light reflects off white paper and can be detected
// by a nearby follower robot whose own emitters are OFF.
//
// Workflow (matches bump-sensor experiment):
//   1.  Power on leader  → WAITING (emitters OFF, motors stopped)
//   2.  Power on follower → follower measures ambient baseline
//   3.  Press leader button A → TARGET_BEACON (emitters ON, stationary)
//   4.  Press follower button A → follower samples target signature
//   5.  Wait for follower to finish sampling
//   6.  Press leader button A again → WAITING (emitters OFF)
//   7.  Follower detects emitters gone, enters WAIT_ON
//   8.  Press leader button B → PRESTART (emitters ON, stationary ~800 ms)
//   9.  Leader auto-transitions → REVERSING (emitters ON, PID straight run)
//  10.  After REVERSE_TIME_MS, leader stops and turns emitters OFF
//  11.  Follower detects loss, stops
//
// Hardware assumptions:
//   - Pololu 3Pi+ 32U4
//   - Line emitter controlled via EMIT_PIN (defined in LineSensors.h)
//     Driving EMIT_PIN LOW (OUTPUT) turns emitters ON;
//     setting EMIT_PIN INPUT turns emitters OFF.
//   - Motors.h, PID.h, Encoders.h present and unchanged from labsheets
//   - count_e0 (right) and count_e1 (left) are ISR encoder counters
// ============================================================

#include "Motors.h"
#include "LineSensors.h"   // only needed for EMIT_PIN and NUM_SENSORS
#include "PID.h"
#include "Encoders.h"
#include "Kinematics.h"

Motors_c motors;
PID_c    left_pid;
PID_c    right_pid;
Kinematics_c pose;

// -------------------- PIN DEFINITIONS --------------------
const int BUTTON_A_PIN = 14;   // toggle stationary emitter / target-beacon mode
const int BUTTON_B_PIN = 30;   // start the reverse run
const int BUTTON_C_PIN = 17;   // dump stored CSV after run

// -------------------- MOTION PARAMETERS --------------------
const uint32_t REVERSE_TIME_MS       = 5000;   // duration of the straight reverse run (ms)
const float    REVERSE_SPEED_DEMAND  = -0.35f; // encoder-counts/ms demand (negative = reverse)

// -------------------- TIMING --------------------
const uint32_t SPEED_EST_MS   = 10;    // wheel-speed estimation period (ms)
const uint32_t CTRL_MS        = 20;    // PID update period (ms)
const float    ALPHA           = 0.2f; // EWA smoothing for speed estimate
const uint32_t PRESTART_MS    = 800;   // hold still with emitters ON before reversing

// -------------------- PWM LIMITS --------------------
const int PWM_MAX_ABS = 60;
const int PWM_FLOOR   = 18;   // minimum PWM when motor is demanded to move

// -------------------- PID GAINS --------------------
// Tuned for straight-line reverse; adjust if robot curves.
const float DRIVE_KP = 15.0f;
const float DRIVE_KI =  0.1f;
const float DRIVE_KD =  0.0f;

// -------------------- STATE MACHINE --------------------
enum LeaderState {
  WAITING,        // idle; emitters OFF
  TARGET_BEACON,  // stationary; emitters ON for follower target capture
  PRESTART,       // stationary; emitters ON; brief hold before run
  REVERSING       // moving backwards; emitters ON; PID straight-line control
};

LeaderState state = WAITING;

uint32_t prestartStartTime = 0;
uint32_t reverseStartTime  = 0;

// Button edge-detection
bool lastButtonAState = HIGH;
bool lastButtonBState = HIGH;
bool lastButtonCState = HIGH;

// Timing
uint32_t lastCtrlTime  = 0;
uint32_t lastSpeedTime = 0;

uint32_t pose_ts = 0;
const uint32_t POSE_MS = 10;

const uint32_t LOG_MS = 50;
const int LOG_SIZE = 100;

struct LogEntry {
  int32_t x_centi;
  int32_t y_centi;
  uint32_t t_ms;
};

LogEntry logBuffer[LOG_SIZE];
int logIndex = 0;
uint32_t lastLogTime = 0;
bool loggingActive = false;
bool logReadyToDump = false;

// Encoder history for speed estimation
long  last_e0 = 0;
long  last_e1 = 0;

// Smoothed wheel speeds (encoder counts per ms)
float right_speed    = 0.0f;
float left_speed     = 0.0f;
float last_speed_e0  = 0.0f;
float last_speed_e1  = 0.0f;

// -------------------- HELPER FUNCTIONS --------------------

int clampInt(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Ensure PWM magnitude is at least floorVal when a non-zero speed is demanded,
// so the motors actually turn rather than stalling at low duty cycles.
int applyPwmFloor(int pwm, float demand, int floorVal) {
  if (fabsf(demand) < 1e-6f) return 0;
  if (abs(pwm) < floorVal) {
    pwm = (demand > 0.0f) ? floorVal : -floorVal;
  }
  return pwm;
}

// Turn the line-sensor IR emitters ON.
// The 3Pi+ EMIT_PIN is active-low: drive LOW to illuminate.
// NOTE: If your board variant drives HIGH to enable, swap the logic here.
void emittersOn() {
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);
}

// Turn the line-sensor IR emitters OFF by floating the pin.
void emittersOff() {
  pinMode(EMIT_PIN, INPUT);
}

void stopRobot() {
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

void updatePoseIfNeeded() {
  uint32_t now = millis();
  if (now - pose_ts >= POSE_MS) {
    pose_ts = now;
    pose.update();
  }
}

void startLogging() {
  logIndex = 0;
  loggingActive = true;
  logReadyToDump = false;
  lastLogTime = millis() - LOG_MS;
}

void finishLogging() {
  loggingActive = false;
  logReadyToDump = true;
}

void logData() {
  if (!loggingActive) return;

  uint32_t now = millis();
  if (now - lastLogTime < LOG_MS) return;
  lastLogTime = now;

  if (logIndex >= LOG_SIZE) return;

  logBuffer[logIndex].x_centi = (int32_t)(pose.x * 100.0f);
  logBuffer[logIndex].y_centi = (int32_t)(pose.y * 100.0f);
  logBuffer[logIndex].t_ms = now;
  logIndex++;
}

void dumpLogCSV() {
  Serial.println(F("i,x_mm,y_mm,ts"));
  for (int i = 0; i < logIndex; i++) {
    Serial.print(i); Serial.print(",");
    Serial.print(logBuffer[i].x_centi / 100.0f, 2); Serial.print(",");
    Serial.print(logBuffer[i].y_centi / 100.0f, 2); Serial.print(",");
    Serial.println(logBuffer[i].t_ms);
  }
}

// Call frequently; updates speed estimate only when SPEED_EST_MS has elapsed.
void updateWheelSpeedsIfNeeded() {
  uint32_t now     = millis();
  uint32_t elapsed = now - lastSpeedTime;
  if (elapsed < SPEED_EST_MS) return;

  lastSpeedTime = now;

  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;
  last_e0 = count_e0;
  last_e1 = count_e1;

  float spd_e0 = (float)delta_e0 / (float)elapsed;
  float spd_e1 = (float)delta_e1 / (float)elapsed;

  right_speed   = ALPHA * spd_e0 + (1.0f - ALPHA) * last_speed_e0;
  left_speed    = ALPHA * spd_e1 + (1.0f - ALPHA) * last_speed_e1;
  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
}

// Call from the REVERSING state; runs PID only when CTRL_MS has elapsed.
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

// Return a short printable name for the current state.
const char* getStateName(LeaderState s) {
  switch (s) {
    case WAITING:       return "WAITING";
    case TARGET_BEACON: return "TGT_BCN";
    case PRESTART:      return "PRESTART";
    case REVERSING:     return "REVERSING";
    default:            return "UNKNOWN";
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_C_PIN, INPUT_PULLUP);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();
  pose.initialise(0.0f, 0.0f, 0.0f);

  last_e0 = count_e0;
  last_e1 = count_e1;
  lastSpeedTime = millis();
  lastCtrlTime  = millis();
  pose_ts = millis();

  // Safe start: emitters OFF, motors stopped.
  emittersOff();
  motors.setPWM(0, 0);
}

// -------------------- LOOP --------------------
void loop() {
  uint32_t now = millis();

  // ---- Read buttons (active LOW with internal pull-ups) ----
  bool buttonAState = digitalRead(BUTTON_A_PIN);
  bool buttonBState = digitalRead(BUTTON_B_PIN);
  bool buttonCState = digitalRead(BUTTON_C_PIN);

  // ---- Button A: toggle TARGET_BEACON while in WAITING ----
  if (lastButtonAState == HIGH && buttonAState == LOW) {
    delay(20); // debounce
    if (digitalRead(BUTTON_A_PIN) == LOW) {
      if (state == WAITING) {
        state = TARGET_BEACON;
      } else if (state == TARGET_BEACON) {
        state = WAITING;
      }
      // Ignored during PRESTART / REVERSING for safety
    }
  }

  // ---- Button B: begin run from WAITING only ----
  if (lastButtonBState == HIGH && buttonBState == LOW) {
    delay(20); // debounce
    if (digitalRead(BUTTON_B_PIN) == LOW) {
      if (state == WAITING) {
        stopRobot();
        prestartStartTime = now;
        state = PRESTART;
      }
    }
  }

  // ---- Button C: dump stored CSV after the run ----
  if (lastButtonCState == HIGH && buttonCState == LOW) {
    delay(20); // debounce
    if (digitalRead(BUTTON_C_PIN) == LOW) {
      if (logReadyToDump) {
        dumpLogCSV();
        logReadyToDump = false;
      }
    }
  }

  // ---- Speed estimation (runs every loop) ----
  updateWheelSpeedsIfNeeded();
  updatePoseIfNeeded();

  // ---- State machine ----
  if (state == WAITING) {
    emittersOff();
    motors.setPWM(0, 0);
  }
  else if (state == TARGET_BEACON) {
    // Emitters ON, stationary — follower can sample target signature.
    emittersOn();
    motors.setPWM(0, 0);
  }
  else if (state == PRESTART) {
    // Emitters ON but stationary, giving the follower time to detect
    // the re-appearance of the signal before the run begins.
    emittersOn();
    motors.setPWM(0, 0);

    if (now - prestartStartTime >= PRESTART_MS) {
      left_pid.reset();
      right_pid.reset();
      reverseStartTime = millis();
      startLogging();
      state = REVERSING;
    }
  }
  else if (state == REVERSING) {
    emittersOn();
    updateReverseControlIfNeeded();

    if (millis() - reverseStartTime >= REVERSE_TIME_MS) {
      stopRobot();
      emittersOff();
      finishLogging();
      state = WAITING;
    }
  }

  logData();

  // ---- Update button history ----
  lastButtonAState = buttonAState;
  lastButtonBState = buttonBState;
  lastButtonCState = buttonCState;

}
