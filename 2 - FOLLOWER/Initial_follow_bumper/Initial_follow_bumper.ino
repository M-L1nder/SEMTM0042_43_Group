// Code Used on ML Robot and ?? Robot
// Just attempting to measure IR using Line sensors on follower

#include "PID.h"
#include "Motors.h"
#include "BumpSensors.h"
#include "Encoders.h"

Motors_c motors;
PID_c left_pid;             // To control the left motor
PID_c right_pid;            // To control the right motor
BumpSensors_c bump_sensors;

unsigned long speed_est_ts; // timestamp for speed estimation - used for pose updates also
#define SPEED_EST_MS 10     // 10ms
unsigned long lastLoop = 0;
#define LOOP_TIME 20


// Speed Params
// Previous counts for speed
long last_e0; // Right
long last_e1; // Left
// previous speeds
float last_speed_e0; // Right
float last_speed_e1; // Left
// most recent speed calculated
float right_speed; // Right
float left_speed; // Left
// smoothing constant
#define alpha 0.2


void setup() {
  Serial.begin(9600);
  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  // Zero Counts and last Speeds
  last_e0 = count_e0;
  last_e1 = count_e1;
  last_speed_e0 = 0;
  last_speed_e1 = 0;

  // Initialise PID Controllers
  left_pid.initialise( 15.0, 0.1, 0.0); // Left motor does start slightly higher pwm than left but these equal controllers worked acceptably
  right_pid.initialise( 15.0, 0.1, 0.0); //kp ki kd
  left_pid.reset();
  right_pid.reset();

  bump_sensors.initialiseForDigital();
  calibrateBumpSensors();
  Serial.println("L,R,diff");
  bump_sensors.startDigitalReadAll();
  lastLoop = millis();
  speed_est_ts = millis();
}

//void loop() {
//
//
//  unsigned long elapsed_time = millis() - speed_est_ts;
//  if (elapsed_time > SPEED_EST_MS) {
//    speedcalc(elapsed_time);
//    speed_est_ts = millis();
//  }
//
//  if (millis() - lastLoop < LOOP_TIME) return;
//  lastLoop = millis();
//  bump_sensors.calcCalibratedDigital();
//
//
//  float L_Cal = bump_sensors.bumpcalibrated[0];
//  float R_Cal = bump_sensors.bumpcalibrated[1];
//  float diff_cal = R_Cal - L_Cal;
//  float mean_cal = 0.5 * (L_Cal + R_Cal);
//
//  // ---- FOLLOW CONTROL ----
//  const float TARGET_IR = 0.20;          // desired average calibrated reading
//  const float SIGNAL_LOST_THRESH = 0.90; // if above this, beacon is probably not seen
//
//  // Outer-loop gains: these create target wheel speeds
//  const float K_DIST = 2;
//  const float K_TURN = 2;
//
//  // Clamp target wheel speeds
//  const float MAX_FWD  = 0.35;
//  const float MAX_TURN = 0.25;
//
//  float left_target = 0.0;
//  float right_target = 0.0;
//
//  if (mean_cal > SIGNAL_LOST_THRESH) {
//    // Beacon probably lost: slow search spin
//    left_target  = -0.15;
//    right_target =  0.15;
//  } else {
//    // Smaller calibrated value = stronger IR
//    // So mean_cal > TARGET_IR => too far => move forward
//    float forward_target = K_DIST * (mean_cal - TARGET_IR);
//
//    // diff_cal = R - L
//    // If diff_cal > 0, beacon is more to the LEFT, so turn left
//    float turn_target = K_TURN * diff_cal;
//
//    // Deadbands to reduce jitter
//    if (abs(mean_cal - TARGET_IR) < 0.03) forward_target = 0.0;
//    if (abs(diff_cal) < 0.03) turn_target = 0.0;
//
//    // Clamp
//    if (forward_target > MAX_FWD)  forward_target = MAX_FWD;
//    if (forward_target < -MAX_FWD) forward_target = -MAX_FWD;
//
//    if (turn_target > MAX_TURN)  turn_target = MAX_TURN;
//    if (turn_target < -MAX_TURN) turn_target = -MAX_TURN;
//
//    // Differential drive targets
//    left_target  = forward_target - turn_target;
//    right_target = forward_target + turn_target;
//  }
//
//  float l_pwm = left_pid.update(left_target, left_speed);
//  float r_pwm = right_pid.update(right_target, right_speed);
//  motors.setPWM(l_pwm, r_pwm);
//
//  Serial.print(L_Cal, 3);
//  Serial.print(",");
//  Serial.print(R_Cal, 3);
//  Serial.print(",");
//  Serial.print(mean_cal, 3);
//  Serial.print(",");
//  Serial.print(diff_cal, 3);
//  Serial.print(",");
//  Serial.print(left_speed, 3);
//  Serial.print(",");
//  Serial.println(right_speed, 3);
//
//
//}

enum FollowMode {
  ALIGNING,
  FOLLOWING
};

FollowMode follow_mode = ALIGNING;

void loop() {
  bump_sensors.serviceDigitalRead();

  unsigned long now = millis();
  if (now - speed_est_ts >= SPEED_EST_MS) {
    unsigned long elapsed = now - speed_est_ts;
    speedcalc(elapsed);
    speed_est_ts = now;
  }

  static float L_filt = 1.0;
  static float R_filt = 1.0;

  if (bump_sensors.sampleReady()) {
    bump_sensors.calcCalibratedFromCurrentReadings();

    float L = bump_sensors.bumpcalibrated[0];
    float R = bump_sensors.bumpcalibrated[1];

    const float beta = 0.35;
    L_filt = (1.0 - beta) * L_filt + beta * L;
    R_filt = (1.0 - beta) * R_filt + beta * R;

    float mean_cal = 0.5 * (L_filt + R_filt);
    float diff_cal = R_filt - L_filt;

    followDistanceOnly(mean_cal);

    Serial.print(L_filt, 3);
    Serial.print(",");
    Serial.print(R_filt, 3);
    Serial.print(",");
    Serial.print(mean_cal, 3);
    Serial.print(",");
    Serial.println(diff_cal, 3);

    bump_sensors.clearSampleReady();
    bump_sensors.startDigitalReadAll();
  }
}

void followDistanceOnly(float mean_cal)
{
  const float TARGET_IR = 0.20;
  const float SIGNAL_LOST_THRESH = 0.90;

  const float K_DIST = 0.40;
  const float MAX_SPEED = 0.70;

  float target_speed = 0.0;

  // If no beacon seen, stop
  if (mean_cal > SIGNAL_LOST_THRESH) {
    target_speed = 0.0;
  } else {
    // Smaller calibrated = stronger IR
    // mean_cal > TARGET_IR => too far => move forward
    float dist_error = mean_cal - TARGET_IR;
    target_speed = K_DIST * dist_error;

    // deadband near target
    if (abs(dist_error) < 0.03) {
      target_speed = 0.0;
    }

    if (target_speed > MAX_SPEED)  target_speed = MAX_SPEED;
    if (target_speed < -MAX_SPEED) target_speed = -MAX_SPEED;
  }

  // BOTH wheels commanded equally
  float l_pwm = left_pid.update(target_speed, left_speed);
  float r_pwm = right_pid.update(target_speed, right_speed);

  motors.setPWM(l_pwm, r_pwm);
}

// ---- STOP ROBOT ----
// Stops the motors, resets PID
void stoprobot() {
  motors.setPWM(0, 0);
  left_pid.reset();  // Clear PID memory
  right_pid.reset();
}


// ---- SPEED CALC ----
// Calculate speeds
void speedcalc(unsigned long elapsed_time) {
  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;

  // Speeds in Counts/ms
  float speed_e0 = float(delta_e0) / float(elapsed_time);
  float speed_e1 = float(delta_e1) / float(elapsed_time);

  // Smoothing
  right_speed = alpha * speed_e0 + (1 - alpha) * last_speed_e0;
  left_speed  = alpha * speed_e1 + (1 - alpha) * last_speed_e1;

  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
  last_e0 = count_e0;
  last_e1 = count_e1;
}

// ---- CALIBRATION ----

void calibrateBumpSensors() {

  // Initialise min / max values
  for (int sensor = 0; sensor < BUMP_NUM_SENSORS; sensor++) {
    bump_sensors.maximum[sensor] = 0;
    bump_sensors.minimum[sensor] = bump_sensors.timeout_us;
  }

  // Timing for calibration
  unsigned long cal_start_time = millis();
  unsigned long calibration_end = cal_start_time + 4000;

  // Optional: ramped spin speed
  float max_target_speed = 0.4;
  float current_target = 0;

  while (millis() < calibration_end) {

    unsigned long current_ms = millis();
    unsigned long elapsed = current_ms - speed_est_ts;

    if (elapsed >= SPEED_EST_MS) {
      speedcalc(elapsed);
      speed_est_ts = current_ms;
      //pose.update();

      // Ramp speed up and down to reduce slip
      unsigned long time_into_cal = current_ms - cal_start_time;

      if (time_into_cal < 1000) {
        current_target = ((float)time_into_cal / 1000.0) * max_target_speed;
      }
      else if (calibration_end - current_ms < 500) {
        current_target = ((float)(calibration_end - current_ms) / 500.0) * max_target_speed;
      }
      else {
        current_target = max_target_speed;
      }

      float l_pwm = left_pid.update(current_target, left_speed);
      float r_pwm = right_pid.update(-current_target, right_speed);
      motors.setPWM(l_pwm, r_pwm);
    }

    // Read bump sensors using digital discharge timing
    bump_sensors.readSensorsDigital();

    // Update min / max
    bump_sensors.updateCalibrationFromReadings();
  }

  // Stop robot
  stoprobot();

  // Compute ranges
  bump_sensors.finaliseCalibration();
}
