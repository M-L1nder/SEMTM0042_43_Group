// ============================================================
// Line_Follower_Reflective_V1.ino
// Pololu 3Pi+ 32U4 — Reflective Line-Sensor Follower
//
// Experiment: detect the leader robot indirectly via IR light
// emitted downward by the leader's line-sensor emitters,
// reflected off white paper, and sensed by the follower's own
// line sensors with the follower's emitters TURNED OFF.
//
// The follower does NOT perform tape line-following.
// It tries to maintain its initial lateral (Y) offset from the
// leader's path while moving forward in a parallel straight line.
//
// ---- Sensing concept ----
// Leader emitters ON → IR reflects off white paper toward follower.
// Follower ADC reads all 5 sensors with its own emitters OFF.
// More IR from the leader → LOWER ADC reading (capacitor discharges faster
// with more incident light in the 3Pi+ reflectance circuit, similar to the
// bump sensor discharge-time principle).
// We convert each raw ADC sample to a "signal" value:
//   signal[i] = clamp( (baseline[i] - raw[i]) / baseline[i] * 100, 0, 100 )
// where baseline was captured with the leader emitters OFF.
// More IR = higher signal value. Units are "% of baseline shift".
//
// ---- 5-sensor spatial features ----
//   totalSignal  = sum of all 5 signals            → fore-aft proximity
//   leftAgg      = weighted sum of left sensors    → lateral energy on left
//   rightAgg     = weighted sum of right sensors   → lateral energy on right
//   centerOfMass = weighted-average position index → lateral position estimate
//                  (0 = far left, 4 = far right)
//   diffLR       = rightAgg - leftAgg              → signed lateral offset
//
// ---- Control strategy ----
//   Target capture records targetTotal, targetDiff, and per-sensor targetSig[].
//   During tracking:
//     - totalSignal vs targetTotal → forward speed demand (closer = slow)
//     - diffLR vs targetDiff       → turn trim demand
//   The follower preserves its starting offset; it does NOT try to centre itself.
//
// ---- Workflow ----
//   1.  Power on follower → measures ambient baseline (leader emitters OFF)
//   2.  Press leader button A → leader emitters ON, stationary
//   3.  Press follower button A → SAMPLING_TARGET (2 s capture at desired offset)
//   4.  Wait for capture to finish → WAIT_OFF
//   5.  Press leader button A again → leader emitters OFF
//   6.  Follower detects loss → WAIT_ON
//   7.  Press leader button B → leader PRESTART then REVERSING (emitters ON)
//   8.  Follower detects emitters reappear → FOLLOWING
//   9.  Follower drives forward, preserving sampled lateral signature
//  10.  Leader stops, emitters OFF → follower detects loss → stops
//
// Hardware assumptions:
//   - Pololu 3Pi+ 32U4
//   - LineSensors.h from labsheet (sensor_pins[], EMIT_PIN, NUM_SENSORS = 5)
//   - Motors.h, PID.h, Encoders.h present and unchanged
//   - count_e0 (right encoder), count_e1 (left encoder) as ISR globals
//   - ADC pin A11, A0, A2, A3, A4 as per LineSensors.h
//
// IMPORTANT ASSUMPTION about EMIT_PIN polarity on 3Pi+:
//   Setting EMIT_PIN as OUTPUT and driving LOW activates the emitters.
//   Setting EMIT_PIN as INPUT (high-impedance) de-activates them.
//   The follower always leaves EMIT_PIN as INPUT so its own emitters
//   stay OFF during all measurements, including the baseline.
//   If your board has reversed polarity, change emittersOff() accordingly.
// ============================================================

#include "Motors.h"
#include "LineSensors.h"   // provides sensor_pins[], EMIT_PIN, NUM_SENSORS
#include "PID.h"
#include "Encoders.h"
#include <math.h>

Motors_c motors;
PID_c    left_pid;
PID_c    right_pid;

// ============================================================
// ==================== TUNING PARAMETERS =====================
// ============================================================

// ---- Timing ----
const uint32_t LOOP_TIME_MS          = 20;    // main loop period
const uint32_t SPEED_EST_MS          = 10;    // wheel-speed estimation period
const uint32_t TARGET_SAMPLE_TIME_MS = 2000;  // duration of target-signature capture
const uint32_t LOST_TIMEOUT_MS       = 300;   // ms before declaring leader lost
const float    ALPHA                 = 0.2f;  // EWA smoothing factor for wheel speed

// ---- Baseline measurement ----
const int      BASELINE_SAMPLES      = 80;    // number of ADC averages for baseline

// ---- Beacon / signal detection thresholds ----
// Set SIGNAL_ON_THRESHOLD so it is comfortably above ambient noise but below
// the signal you observe when the leader emitters are ON at the test distance.
// Read the serial log during baseline and target-capture to calibrate these.
const float    SIGNAL_ON_THRESHOLD   = 5.0f;  // totalSignal above this → leader present
const float    SIGNAL_OFF_THRESHOLD  = 2.0f;  // totalSignal below this → leader absent
const int      REQUIRED_ON_COUNT     = 3;     // consecutive samples to declare ON
const int      REQUIRED_OFF_COUNT    = 3;     // consecutive samples to declare OFF

// ---- Target-band built from observed variation during capture ----
const float    BAND_MARGIN_FRACTION  = 0.20f; // ±20% of observed variation
const float    MIN_BAND_MARGIN       = 1.5f;  // minimum absolute margin

// ---- Forward speed control ----
// totalError = targetTotal - currentTotal (positive → leader has moved ahead → drive faster)
const float    BASE_CHASE_SPEED      = 0.18f; // minimum demand when chasing
const float    SPEED_GAIN            = 0.005f;// extra speed per unit of totalError
const float    MAX_FWD_SPEED_DEMAND  = 0.40f; // hard cap on forward speed
const float    TARGET_DEADBAND       = 1.5f;  // ignore totalError within this band

// ---- Lateral / turn control ----
// diffError = targetDiff - currentDiff  (positive → robot is offset too far left → turn right)
const float    TURN_TRIM_GAIN        = 0.008f;// converts diffError → speed differential
const float    MAX_TURN_SPEED_DEMAND = 0.10f; // hard cap on turn component
const float    DIFF_DEADBAND         = 2.0f;  // ignore diffError within this band
// If the robot enters ALIGNING mode, it uses a fixed spot-turn:
const float    ALIGN_TURN_SPEED      = 0.15f; // each wheel ±this during alignment
const float    DIFF_ENTER_ALIGN      = 10.0f; // enter ALIGNING if |diffError| > this
const float    DIFF_EXIT_ALIGN       = 3.0f;  // leave ALIGNING if |diffError| < this
// Suppress turn trim when strongly chasing forward:
const float    FORWARD_PRIORITY_ERR  = 8.0f;  // above this totalError, zero the trim
const float    MAX_TURN_RATIO        = 0.35f; // turn demand ≤ 35% of forward demand

// ---- Drive sign ----
// If the follower drives backwards instead of forwards, flip FOLLOW_SIGN to -1.
const float    FOLLOW_SIGN           = 1.0f;
// If diffError produces turns in the wrong direction, flip TURN_SIGN.
const float    TURN_SIGN             = -1.0f;

// ---- PWM limits ----
const int      PWM_MAX_ABS           = 60;
const int      PWM_FLOOR_FWD         = 18;
const int      PWM_FLOOR_TURN        = 20;

// ---- PID gains ----
const float    DRIVE_KP              = 15.0f;
const float    DRIVE_KI              =  0.1f;
const float    DRIVE_KD              =  0.0f;

// ---- Spatial weights for 5 sensors (indices 0=DN1 … 4=DN5, left to right) ----
// These define how much each sensor contributes to the left / right aggregates
// and to the centre-of-mass calculation. Adjust if your sensor geometry differs.
//   Sensor layout (3Pi+ looking down from robot's perspective):
//   DN1(0)  DN2(1)  DN3(2)  DN4(3)  DN5(4)
//   far-L   mid-L   centre  mid-R   far-R
const float    POS_WEIGHT[NUM_SENSORS] = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f }; // for CoM
const float    LEFT_MASK[NUM_SENSORS]  = { 1.0f, 0.8f, 0.2f, 0.0f, 0.0f }; // left aggregate
const float    RIGHT_MASK[NUM_SENSORS] = { 0.0f, 0.0f, 0.2f, 0.8f, 1.0f }; // right aggregate

// ============================================================
// ==================== END TUNING PARAMETERS =================
// ============================================================

// -------------------- BUTTON --------------------
const int BUTTON_A_PIN = 14;

// -------------------- STATE MACHINE --------------------
enum RobotState {
  WAITING_FOR_TARGET_BUTTON,  // boot: baseline measured, waiting for button
  SAMPLING_TARGET,            // recording target signature (leader stationary, emitters ON)
  WAITING_FOR_BEACON_OFF,     // target done: waiting for leader emitters to go OFF
  WAITING_FOR_BEACON_ON,      // waiting for leader to restart (emitters back ON)
  FOLLOWING                   // active tracking
};

enum TrackMode {
  SEARCHING,  // signal not locked
  ALIGNING,   // spot-turning to correct large lateral error
  DRIVING     // forward with gentle trim
};

RobotState robotState = WAITING_FOR_TARGET_BUTTON;
TrackMode  trackMode  = SEARCHING;

// -------------------- TIMING --------------------
uint32_t lastLoop        = 0;
uint32_t speed_est_ts    = 0;
uint32_t targetSampleStart = 0;
uint32_t beaconLostStart   = 0;

// -------------------- ENCODER SPEED --------------------
long  last_e0       = 0;
long  last_e1       = 0;
float right_speed   = 0.0f;
float left_speed    = 0.0f;
float last_speed_e0 = 0.0f;
float last_speed_e1 = 0.0f;

// -------------------- SENSOR DATA --------------------
// Raw ADC readings (0–1023 on 3Pi+)
float raw[NUM_SENSORS];

// Ambient baseline (leader emitters OFF)
float baseline[NUM_SENSORS];

// Per-sensor signal: 0–100, higher = more leader IR
float sig[NUM_SENSORS];

// Derived spatial features
float totalSignal = 0.0f;
float leftAgg     = 0.0f;
float rightAgg    = 0.0f;
float diffLR      = 0.0f;     // rightAgg - leftAgg
float centerOfMass = 2.0f;   // weighted average position (0–4)

// -------------------- TARGET SIGNATURE --------------------
float targetSig[NUM_SENSORS]; // per-sensor target
float targetTotal = 0.0f;
float targetDiff  = 0.0f;
float lowerThreshold = 0.0f;
float upperThreshold = 0.0f;

// Accumulators during capture
float  tgtSumTotal         = 0.0f;
float  tgtSumDiff          = 0.0f;
float  tgtSumSig[NUM_SENSORS];
int    targetSampleCount   = 0;
float  minTotalSeen        =  100000.0f;
float  maxTotalSeen        = -100000.0f;

// -------------------- CONTROL STATE --------------------
bool  lastButtonState = HIGH;
float smoothedDrive   = 0.0f;
int   turnDir         = 1;    // latched turn direction (+1 or -1)
int   onCount         = 0;    // hysteresis counter for BEACON_ON detection
int   offCount        = 0;    // hysteresis counter for BEACON_OFF detection

// -------------------- HELPER FUNCTIONS --------------------

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

int applyPwmFloor(int pwm, float demand, int floorVal) {
  if (fabsf(demand) < 1e-6f) return 0;
  if (abs(pwm) < floorVal) pwm = (demand > 0.0f) ? floorVal : -floorVal;
  return pwm;
}

void stopRobot() {
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

// Ensure the follower's own IR emitters stay OFF.
// Call once in setup and never re-enable during the experiment.
void ensureEmittersOff() {
  // High-impedance input: emitters are not driven.
  // On 3Pi+, EMIT_PIN low (output) = ON; INPUT = OFF.
  pinMode(EMIT_PIN, INPUT);
}

// -------------------- WHEEL SPEED ESTIMATION --------------------

void speedcalc(unsigned long elapsed) {
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

void updateWheelSpeedsIfNeeded() {
  unsigned long elapsed = millis() - speed_est_ts;
  if (elapsed >= SPEED_EST_MS) {
    speedcalc(elapsed);
    speed_est_ts = millis();
  }
}

// -------------------- LINE SENSOR READING --------------------

// Read all 5 ADC sensors with emitters OFF (follower emitters must already be off).
// Stores raw ADC values into raw[].
void readRawSensors() {
  // Emitters are kept OFF (INPUT) throughout; no need to toggle.
  for (int i = 0; i < NUM_SENSORS; i++) {
    raw[i] = (float)analogRead(sensor_pins[i]);
  }
}

// Convert raw ADC readings to signal values using stored baseline.
// signal = clamp( (baseline - raw) / baseline * 100, 0, 100 )
// More leader IR hitting sensor → lower ADC (capacitor discharges faster) → higher signal.
void computeSignals() {
  totalSignal  = 0.0f;
  leftAgg      = 0.0f;
  rightAgg     = 0.0f;
  float posSum = 0.0f;
  float sigSum = 0.0f;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (baseline[i] < 1.0f) {
      sig[i] = 0.0f;
    } else {
      float s = 100.0f * (baseline[i] - raw[i]) / baseline[i];
      if (s < 0.0f)   s = 0.0f;
      if (s > 100.0f) s = 100.0f;
      sig[i] = s;
    }
    totalSignal += sig[i];
    leftAgg     += LEFT_MASK[i]  * sig[i];
    rightAgg    += RIGHT_MASK[i] * sig[i];
    posSum      += POS_WEIGHT[i] * sig[i];
    sigSum      += sig[i];
  }

  diffLR = rightAgg - leftAgg;

  // Centre-of-mass: weighted average position (0–4)
  if (sigSum > 1.0f) {
    centerOfMass = posSum / sigSum;
  } else {
    centerOfMass = 2.0f; // default to centre when no signal
  }
}

// Full update: read sensors then compute derived features.
void updateLineSensors() {
  readRawSensors();
  computeSignals();
}

// -------------------- BASELINE MEASUREMENT --------------------

void measureBaseline() {
  Serial.println("# Measuring ambient baseline — keep leader emitters OFF ...");

  for (int i = 0; i < NUM_SENSORS; i++) baseline[i] = 0.0f;

  for (int n = 0; n < BASELINE_SAMPLES; n++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      baseline[i] += (float)analogRead(sensor_pins[i]);
    }
    delay(5);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    baseline[i] /= (float)BASELINE_SAMPLES;
  }

  Serial.print("# Baseline (DN1..DN5): ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(baseline[i], 1);
    if (i < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println();
  Serial.println("# Baseline done. Turn leader emitters ON, then press follower button A.");
}

// -------------------- TARGET SAMPLING --------------------

void startTargetSampling() {
  targetSampleStart = millis();
  tgtSumTotal       = 0.0f;
  tgtSumDiff        = 0.0f;
  targetSampleCount = 0;
  minTotalSeen      =  100000.0f;
  maxTotalSeen      = -100000.0f;
  for (int i = 0; i < NUM_SENSORS; i++) tgtSumSig[i] = 0.0f;

  Serial.println("# Sampling target signature ...");
}

void accumulateTargetSample() {
  // Only accumulate when the leader signal is actually present.
  if (totalSignal < SIGNAL_ON_THRESHOLD) return;

  for (int i = 0; i < NUM_SENSORS; i++) tgtSumSig[i] += sig[i];
  tgtSumTotal       += totalSignal;
  tgtSumDiff        += diffLR;
  targetSampleCount++;

  if (totalSignal < minTotalSeen) minTotalSeen = totalSignal;
  if (totalSignal > maxTotalSeen) maxTotalSeen = totalSignal;
}

void finishTargetSampling() {
  if (targetSampleCount < 1) targetSampleCount = 1; // guard divide-by-zero

  targetTotal = tgtSumTotal / (float)targetSampleCount;
  targetDiff  = tgtSumDiff  / (float)targetSampleCount;
  for (int i = 0; i < NUM_SENSORS; i++) {
    targetSig[i] = tgtSumSig[i] / (float)targetSampleCount;
  }

  float spread     = maxTotalSeen - minTotalSeen;
  float bandMargin = spread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  Serial.println("# Target sampling complete:");
  Serial.print("# targetTotal="); Serial.print(targetTotal, 2);
  Serial.print(" targetDiff=");   Serial.print(targetDiff,  2);
  Serial.print(" lower=");        Serial.print(lowerThreshold, 2);
  Serial.print(" upper=");        Serial.println(upperThreshold, 2);
  Serial.print("# targetSig: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(targetSig[i], 1);
    if (i < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println();
  Serial.println("# Now press leader button A (emitters OFF), then button B (start run).");
}

// -------------------- MOTOR DRIVE --------------------

// Apply PID and PWM floor, then drive both wheels.
void driveAtDemands(float leftDemand, float rightDemand) {
  leftDemand  = clampFloat(leftDemand,  -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);
  rightDemand = clampFloat(rightDemand, -MAX_FWD_SPEED_DEMAND, MAX_FWD_SPEED_DEMAND);

  leftDemand  *= FOLLOW_SIGN;
  rightDemand *= FOLLOW_SIGN;

  float l_pwm_f = left_pid.update(leftDemand, left_speed);
  float r_pwm_f = right_pid.update(rightDemand, right_speed);

  int l_pwm = clampInt((int)l_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);
  int r_pwm = clampInt((int)r_pwm_f, -PWM_MAX_ABS, PWM_MAX_ABS);

  bool turning = (fabsf(leftDemand - rightDemand) > 0.01f);
  int floorL = turning ? PWM_FLOOR_TURN : PWM_FLOOR_FWD;
  int floorR = turning ? PWM_FLOOR_TURN : PWM_FLOOR_FWD;

  l_pwm = applyPwmFloor(l_pwm, leftDemand, floorL);
  r_pwm = applyPwmFloor(r_pwm, rightDemand, floorR);

  motors.setPWM(l_pwm, r_pwm);
}

// -------------------- STATE NAME HELPERS --------------------

const char* getStateName(RobotState s) {
  switch (s) {
    case WAITING_FOR_TARGET_BUTTON: return "WAIT_TGT";
    case SAMPLING_TARGET:           return "SAMPLING";
    case WAITING_FOR_BEACON_OFF:    return "WAIT_OFF";
    case WAITING_FOR_BEACON_ON:     return "WAIT_ON";
    case FOLLOWING:                 return "FOLLOWING";
    default:                        return "UNKNOWN";
  }
}

const char* getTrackName(TrackMode m) {
  switch (m) {
    case SEARCHING: return "SEARCH";
    case ALIGNING:  return "ALIGN";
    case DRIVING:   return "DRIVE";
    default:        return "UNK";
  }
}

// -------------------- CSV SERIAL LOGGING --------------------
// Columns (CSV, easy to import into MATLAB/Excel):
//   time_ms, state, trackMode,
//   raw0..raw4,
//   sig0..sig4,
//   totalSignal, leftAgg, rightAgg, diffLR, centerOfMass,
//   tgt_total, tgt_diff, tgt_sig0..tgt_sig4,
//   totalError, diffError,
//   cmdDrive, cmdTurn,
//   left_spd, right_spd

void printCSVHeader() {
  Serial.println(
    "time_ms,state,trackMode,"
    "raw0,raw1,raw2,raw3,raw4,"
    "sig0,sig1,sig2,sig3,sig4,"
    "totalSignal,leftAgg,rightAgg,diffLR,centerOfMass,"
    "tgt_total,tgt_diff,tgt_sig0,tgt_sig1,tgt_sig2,tgt_sig3,tgt_sig4,"
    "totalError,diffError,"
    "cmdDrive,cmdTurn,"
    "left_spd,right_spd"
  );
}

void printCSV(float totalError, float diffError, float cmdDrive, float cmdTurn) {
  Serial.print(millis());             Serial.print(",");
  Serial.print(getStateName(robotState)); Serial.print(",");
  Serial.print(getTrackName(trackMode));  Serial.print(",");

  // Raw ADC
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(raw[i], 0); Serial.print(",");
  }

  // Baseline-corrected signals
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sig[i], 2); Serial.print(",");
  }

  // Spatial features
  Serial.print(totalSignal,  2); Serial.print(",");
  Serial.print(leftAgg,      2); Serial.print(",");
  Serial.print(rightAgg,     2); Serial.print(",");
  Serial.print(diffLR,       2); Serial.print(",");
  Serial.print(centerOfMass, 3); Serial.print(",");

  // Target signature
  Serial.print(targetTotal, 2); Serial.print(",");
  Serial.print(targetDiff,  2); Serial.print(",");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(targetSig[i], 2); Serial.print(",");
  }

  // Error terms
  Serial.print(totalError, 2); Serial.print(",");
  Serial.print(diffError,  2); Serial.print(",");

  // Commands
  Serial.print(cmdDrive, 4); Serial.print(",");
  Serial.print(cmdTurn,  4); Serial.print(",");

  // Wheel speeds
  Serial.print(left_speed,  4); Serial.print(",");
  Serial.println(right_speed, 4);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // Ensure follower emitters are OFF before anything else.
  ensureEmittersOff();

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  left_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  right_pid.initialise(DRIVE_KP, DRIVE_KI, DRIVE_KD);
  left_pid.reset();
  right_pid.reset();

  last_e0 = count_e0;
  last_e1 = count_e1;
  speed_est_ts = millis();

  // Initialise sensor pins for ADC input.
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensor_pins[i], INPUT_PULLUP);
  }

  // Step 1: measure ambient baseline (leader emitters must be OFF).
  measureBaseline();

  printCSVHeader();

  robotState = WAITING_FOR_TARGET_BUTTON;
  trackMode  = SEARCHING;
  lastLoop   = millis();
}

// -------------------- LOOP --------------------
void loop() {
  // ---- Wheel speed estimation (runs at SPEED_EST_MS regardless of loop gate) ----
  updateWheelSpeedsIfNeeded();

  // ---- Main loop gate ----
  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  // ---- Always read sensors first ----
  updateLineSensors();

  bool buttonState = digitalRead(BUTTON_A_PIN);

  // Variables for logging (updated per state)
  float totalError = 0.0f;
  float diffError  = 0.0f;
  float cmdDrive   = 0.0f;
  float cmdTurn    = 0.0f;

  // ==================================================================
  // STATE: WAITING_FOR_TARGET_BUTTON
  // Boot state. Baseline has been measured. Robot waits for operator
  // to press button A after placing both robots at the desired offset
  // and turning the leader emitters ON.
  // ==================================================================
  if (robotState == WAITING_FOR_TARGET_BUTTON) {
    stopRobot();

    if (lastButtonState == HIGH && buttonState == LOW) {
      delay(20);
      if (digitalRead(BUTTON_A_PIN) == LOW) {
        startTargetSampling();
        robotState = SAMPLING_TARGET;
      }
    }
  }

  // ==================================================================
  // STATE: SAMPLING_TARGET
  // Leader is stationary with emitters ON. Follower records the
  // 5-sensor spatial pattern for 2 seconds.
  // ==================================================================
  else if (robotState == SAMPLING_TARGET) {
    stopRobot();
    accumulateTargetSample();

    if (millis() - targetSampleStart >= TARGET_SAMPLE_TIME_MS) {
      finishTargetSampling();
      // Skip WAIT_OFF: go straight to WAIT_ON so the operator can
      // disconnect the USB cable and press follower button A to start
      // without needing to cycle the leader emitters off and on again.
      onCount    = 0;
      robotState = WAITING_FOR_BEACON_ON;
      Serial.println("# Ready. Disconnect cable, press leader B then follower A to start.");
    }
  }

  // ==================================================================
  // STATE: WAITING_FOR_BEACON_OFF
  // Target captured. Now wait for the operator to turn the leader
  // emitters OFF (press leader button A). Use hysteresis counter.
  // ==================================================================
  else if (robotState == WAITING_FOR_BEACON_OFF) {
    stopRobot();

    if (totalSignal < SIGNAL_OFF_THRESHOLD) {
      offCount++;
    } else {
      offCount = 0;
    }

    if (offCount >= REQUIRED_OFF_COUNT) {
      offCount   = 0;
      onCount    = 0;
      robotState = WAITING_FOR_BEACON_ON;
    }
  }

  // ==================================================================
  // STATE: WAITING_FOR_BEACON_ON
  // Emitters are off. Wait for the leader to restart (button B on
  // leader triggers PRESTART then REVERSING, turning emitters back ON).
  //
  // DIRECT-START SHORTCUT: pressing follower button A while in this
  // state immediately launches FOLLOWING using the already-captured
  // target signature. Use this when you cannot keep the serial cable
  // attached during the run — complete target sampling, disconnect
  // cable, position robots, press leader B then immediately press
  // follower A to start tracking without waiting for auto-detection.
  // ==================================================================
  else if (robotState == WAITING_FOR_BEACON_ON) {
    stopRobot();

    // Auto-detect: leader emitters reappear
    if (totalSignal > SIGNAL_ON_THRESHOLD) {
      onCount++;
    } else {
      onCount = 0;
    }

    bool autoStart = (onCount >= REQUIRED_ON_COUNT);

    // Manual direct-start via button A
    bool manualStart = false;
    if (lastButtonState == HIGH && buttonState == LOW) {
      delay(20);
      if (digitalRead(BUTTON_A_PIN) == LOW) {
        manualStart = true;
      }
    }

    if (autoStart || manualStart) {
      onCount          = 0;
      left_pid.reset();
      right_pid.reset();
      smoothedDrive    = 0.0f;
      beaconLostStart  = 0;
      turnDir          = 1;
      trackMode        = DRIVING;
      robotState       = FOLLOWING;
      Serial.println(manualStart ? "# FOLLOWING: manual start" : "# FOLLOWING: auto start");
    }
  }

  // ==================================================================
  // STATE: FOLLOWING
  // Active tracking. Two sub-modes:
  //   ALIGNING — spot-turn when lateral error is too large
  //   DRIVING  — forward with gentle turn trim
  // ==================================================================
  else if (robotState == FOLLOWING) {

    // ---- Signal lost? ----
    if (totalSignal < SIGNAL_OFF_THRESHOLD) {
      if (beaconLostStart == 0) beaconLostStart = millis();

      if (millis() - beaconLostStart > LOST_TIMEOUT_MS) {
        // Leader has gone — stop and wait for it to reappear.
        stopRobot();
        trackMode  = SEARCHING;
        onCount    = 0;
        robotState = WAITING_FOR_BEACON_ON;
      } else {
        // Brief dropout: hold last manoeuvre to avoid jerky stop.
        // Maintain last turn direction in case it was aligning.
        driveAtDemands(-turnDir * ALIGN_TURN_SPEED * 0.5f,
                        turnDir * ALIGN_TURN_SPEED * 0.5f);
      }
    }
    else {
      // Signal present; reset lost timer.
      beaconLostStart = 0;

      // ---- Compute error terms ----
      totalError = targetTotal - totalSignal;
      // Positive totalError → too far from leader → need to drive forward faster.

      diffError  = targetDiff - diffLR;
      // Positive diffError → current right-left is less than target → steer right.

      // ---- Deadbands ----
      float activeDiffError = (fabsf(diffError) > DIFF_DEADBAND) ? diffError : 0.0f;

      // ---- Check if we should enter ALIGNING ----
      if (trackMode != ALIGNING && fabsf(diffError) > DIFF_ENTER_ALIGN) {
        trackMode = ALIGNING;
        turnDir   = (diffError > 0.0f) ? 1 : -1; // +1 = turn right
      }

      if (trackMode == ALIGNING) {
        // Spot turn until lateral error is small.
        if (fabsf(diffError) < DIFF_EXIT_ALIGN) {
          trackMode = DRIVING;
          stopRobot();
          cmdDrive = 0.0f;
          cmdTurn  = 0.0f;
        } else {
          // Spot turn: right wheel forward, left wheel back (or vice versa).
          cmdDrive = 0.0f;
          cmdTurn  = (float)turnDir * ALIGN_TURN_SPEED;
          driveAtDemands(-cmdTurn, cmdTurn);
        }
      }
      else {
        // ---- DRIVING mode: forward with trim ----

        // Forward speed from total-signal error.
        float desiredDrive = 0.0f;
        float activeTotalError = (fabsf(totalError) > TARGET_DEADBAND)
                                  ? totalError - TARGET_DEADBAND : 0.0f;
        if (activeTotalError > 0.0f) {
          desiredDrive = BASE_CHASE_SPEED + SPEED_GAIN * activeTotalError;
        }
        desiredDrive = clampFloat(desiredDrive, 0.0f, MAX_FWD_SPEED_DEMAND);

        // Turn trim from diff error (suppress when strongly chasing).
        float desiredTurn = 0.0f;
        if (activeTotalError <= FORWARD_PRIORITY_ERR) {
          desiredTurn = TURN_SIGN * TURN_TRIM_GAIN * activeDiffError;
          desiredTurn = clampFloat(desiredTurn, -MAX_TURN_SPEED_DEMAND, MAX_TURN_SPEED_DEMAND);

          // Never let turn dominate forward motion.
          float maxTurnAllowed = MAX_TURN_RATIO * desiredDrive;
          desiredTurn = clampFloat(desiredTurn, -maxTurnAllowed, maxTurnAllowed);
        }

        cmdDrive = desiredDrive;
        cmdTurn  = desiredTurn;

        float leftDemand  = desiredDrive - desiredTurn;
        float rightDemand = desiredDrive + desiredTurn;

        smoothedDrive = desiredDrive;
        driveAtDemands(leftDemand, rightDemand);
      }
    }
  }

  // ---- Update button history ----
  lastButtonState = buttonState;

  // ---- CSV log every loop ----
  printCSV(totalError, diffError, cmdDrive, cmdTurn);
}
