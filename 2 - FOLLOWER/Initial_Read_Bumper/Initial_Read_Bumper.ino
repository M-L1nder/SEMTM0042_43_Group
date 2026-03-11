// Code Used on ML Robot and ?? Robot
// Just attempting to measure IR using Line sensors on follower
#include "PID.h"
#include "Motors.h"
#include "BumpSensors.h"
#include "Encoders.h"

Motors_c motors;
BumpSensors_c bump_sensors;
PID_c left_pid;             // To control the left motor
PID_c right_pid;            // To control the right motor


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
  lastLoop = millis();
  speed_est_ts = millis();
}

void loop() {
  bump_sensors.calcCalibratedDigital();

  unsigned long elapsed_time = millis() - speed_est_ts;
  if (elapsed_time > SPEED_EST_MS) {
    speedcalc(elapsed_time);
    speed_est_ts = millis();
  }
  
  if(millis() - lastLoop < LOOP_TIME) return;
  lastLoop = millis();
  float L = bump_sensors.readings[0];
  float R = bump_sensors.readings[1];
  float diff = R - L;

  float L_Cal = bump_sensors.bumpcalibrated[0];
  float R_Cal = bump_sensors.bumpcalibrated[1];

  Serial.print(L);
  Serial.print(",");
  Serial.print(R);
  Serial.print(",");
  Serial.print(100*L_Cal);
  Serial.print(",");
  Serial.println(100*R_Cal);


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
//
//void setup()
//{
//  Serial.begin(9600);
//
//  motors.initialise();
//  bump_sensors.initialiseForADC();
//
//
//  delay(1000);
//  Serial.println("Ready");
//  delay(1000);
//  lastLoop = millis();
//}
//
//void loop()
//{
//  if(millis() - lastLoop < LOOP_TIME) return;
//  lastLoop += LOOP_TIME;
//
//  bump_sensors.readSensorsADC();
//
//  float raw = bump_sensors.readings[0];
//  
//  //float R = bump_sensors.readings[1];
//  //float diff = R - L;
//
//
//  if(raw > 200){
//    motors.setPWM(50,50);  // move forward toward beacon
//  }
//  else{
//    motors.setPWM(0,0);    // no beacon
//  }
//
//  
//  // Logging
//  Serial.println(raw);// Serial.print(",");
//  //Serial.print(R); Serial.print(",");
//  //Serial.println(diff); 
////  Serial.print(pwmL); Serial.print(",");
////  Serial.println(pwmR);
//}
