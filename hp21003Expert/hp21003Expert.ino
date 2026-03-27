// ---------------- ROBOTICS INDIVIDUAL ASSESSMENT: HP21003 ----------------//
// #########################################################################//
//                                                                          //
// CONTENTS AND OVERVIEW ---------------------------------------------------//
// A finite state machine was implemented for the robot so that it can:     //
// *1. Calibrate magneotometer and line sensors on startup.                 //
// *2. Serach through a series of waypoints to identify a magneitc puck.    // 
// *3. Upon detecting the puck, it then retreats, and reposisiotns behind   //
//     behind the puck and pushes it back to the origin.                    //
// *4. Then returns home and repeats until timer ends.                      //

//FSM STATES -------------------------------------------------------------- // 
// CALIBRATE - Spins and activates sensors and computes calibration values. //
// COOLDOWN - A small paus after calibration before alignment turn.         //
// ROUTE_START - Loads the next waypoint and initiates the turn towards it. //
// TURN_TO - Exectues a PID controlled turn to a target angle.              //
// TRAVEL_TO - Drive towards a target x,y  coordinate with heading          //
//             corrections.                                                 //
// NEXT_WP - Move to the next way point index in route array.               //
// PUSH_PUCK - (UNUSED) Used to move forwards until line is hit.           //
// RETREAT - Reverse away from puck before relaigning to a point behind puck//
// PUSH_HOME - Move forward and puch the puck home, advance to resetting at //
//             home.                                                        //
// STOP_FOUND - (UNUSED) Stop robot and go back home if line detected       //
//  HOME_RETURN - Wait at home for 4 seconds, reset and restart the routine.//
// STOP - Hard Stop (when timer ends or a debig test is complete)           //

//==========================================================================//
//                        SECTION A: GLOBALS, VARIABLES AND LBRARIES        //
//==========================================================================//

# include "Motors.h"
# include "Encoders.h"   
# include "Kinematics.h" 
# include "PID.h"
# include "Magnetometer.h"
# include "LineSensors.h"
# include "OLED.h"

# include <Wire.h>
# include <LIS3MDL.h>

//==========================================================================//
//                        2. OBJECTS,PID AND HARDWARE                       //
//==========================================================================//
OLED_c display(1, 30, 0, 17, 13);
Motors_c motors;
Kinematics_c pose;
Magnetometer_c magnetometer;
LineSensors_c line_sensors;

PID_c left_pid;
PID_c right_pid;
PID_c turn;
PID_c heading;

// REQUIRED PINS --------------------------------------------------------- //

# define EMIT_PIN 11
# define LED_PIN 13
# define BUZZER_PIN 6

//==========================================================================//
//                        3. FSM SET UP                                     //
//==========================================================================//
// Main Robot States
enum RobotState { CALIBRATE, COOLDOWN, ROUTE_START, TURN_TO, TRAVEL_TO,
                  NEXT_WP, PUSH_PUCK, HOME_RETURN, STOP_FOUND, STOP,
                  RETREAT, PUSH_HOME};
RobotState state = CALIBRATE;

//==========================================================================//
//                        4. HOME ASSIGNMENTS                               //
//==========================================================================//

// - HOME LOCATION CONFIG - //
static const float Home_x = 0.0f;
static const float Home_y = 0.0f;

// - HOME WAITING TIME - (after reaching home before restarting route) //
static const unsigned long HOME_WAIT_MS = 4000;
unsigned long home_wait_start = 0;

// (Bool to make robot go to origin
bool go_home = false;

// --- MAG DRIFT CORRECTION VARIABLE ---
float initial_mag_heading = 0.0f;

//==========================================================================//
//                        4. DEBUG FLAGS                                    //
//==========================================================================//
bool debug_mode = false;
bool debug_mag = false;
bool debug_line = false;

//==========================================================================//
//                        5. DEBUG CONTROL VARIABLES                       //
//==========================================================================//

enum TestMode {TEST_ROUTE, TEST_TURN, TEST_TRAVEL};
TestMode test_mode = TEST_TRAVEL;

bool test_start = false;
bool test_running = false;

float test_x = 200.0f;
float test_y = 0.0f;
float turn_test = (90 * PI / 180);

//==========================================================================//
//                       6. TIMING VARIABLES (FOR SPEED EST)                //
//==========================================================================//

#define POSE_MS 20          // Update the pose readings every 20 ms
#define SPEED_MS 10         // Update the speed readings every 10 ms
#define PID_MS 20           // Update the PID readings every 20 ms

unsigned long pose_ts = 0;
unsigned long speed_ts = 0;
unsigned long pid_ts = 0;

//==========================================================================//
//                        7. SPEED MEASUREMENT                              //
//==========================================================================//

#define ALPHA 0.2           // Filtering number to create clean speed measurement

long last_e0;               // The last right encoder count
long last_e1;               // The last left encoder count

float speed_e0 = 0.0;       // unfiltered wheel speed right
float speed_e1 = 0.0;       // unfiltered wheel speed left
float speed_e0_filt;        // filtered wheel speed right
float speed_e1_filt;        // filtered wheel speed left

//==========================================================================//
//                        8. CALIBRATION (MAG + LINE VARIABLES)             //
//==========================================================================//

// - CALIBRATION TIMINGS - //
const unsigned long CAL_MS = 4000; // Spin duration for calibration
const unsigned long CLDWN_MS = 500; // Cooldown duration post calibration
unsigned long cal_start = 0;   
unsigned long cal_PID_ts = 0;
unsigned long cooldown_start = 0;

// - CALIBRATION PID DEMAND - //
const float CAL_DMD = 0.4f; //Calibration spin demand

// - MAGNETIC CALIBRATION VARIABLES - //
# define NUM_AXIS 3
// Variables to store calibration constants.
float minimum[ NUM_AXIS ];
float maximum[ NUM_AXIS ];
float mag_range[ NUM_AXIS ];
float mag_offset[NUM_AXIS];
float mag_scaling[NUM_AXIS];
float magcalibrated[NUM_AXIS];

bool mag_calibrated = false;

// - LINE CALIBRATION VARIABLES - //

float line_range[NUM_SENSORS];
bool line_calibrated = false;

// - realignment after calibration - //
bool cal_realign = false;

//==========================================================================//
//                        9. DETECTIION AND THRESHOLD (MAG + LIN)           //
//==========================================================================//

// - DETECTION VARIABLES - //
const unsigned long DEBUG_MS = 200; 
const float MAG_THRESHOLD = 2.9f;    // Normalied Mag magnitude to trigger detection
const float LINE_THRESHOLD = 0.78f;  // Nomrlaised line value to trigger line sensing

const unsigned long MAG_HOLD_MS = 20; // Threshold must be exceeded for at least this long.
// Magnetic signal should be above the threshold for this long
unsigned long mag_start = 0;

bool mag_found = false;

//==========================================================================//
//                        10. TURN CONTROL VARIABLES                         //
//==========================================================================//

#define TURN_TOL_RAD 0.003   // A turning tolerence of 0.002 gives occasional jitter capret but it does on table
#define TURN_MIN      0.20    // A minimum rotation gain
#define TURN_MAX      0.40    // A maximum turn gain 

bool turning = false;

float target_angle = 0.0f;
float start_theta = 0.0f; // quick use for debugging - remove latr if needed 

//==========================================================================//
//                        11. TRAVEL CONTROL VARIABLES                       //
//==========================================================================//

// ---------------- Travel Related Parameters ------------------ //

# define TRAVEL_TOL      5.0
# define TRAVEL_SPD      0.5

// define HEADING_TURN_MAX = 0.8f

bool travelling = false;

float target_y = 0.0;
float target_x = 200.0;
float desired_heading = 0.0;
float theta_L = 0.0f; 
float turn_ctrl = 0.0f;

//==========================================================================//
//                        12. WAYPOINT CONTROL VARIABLES                    //
//==========================================================================//

# define NUM_WAYPOINTS 7
# define route_length 7

float target_x_num[NUM_WAYPOINTS] = {0.0f, 263.5f, 210.5f, 265.5f, 111.5f, -2.5f, 84.5f};
float target_y_num[NUM_WAYPOINTS] = {0.0f, -60.5f, -248.5f, -416.5f, -394.0f, -339.5f, -158.5f };

int current_target = 0;
int route_stage = 0;
int route_order[route_length] = {1, 2, 3, 4, 5, 6, 0}; // order of waypoints

bool route_started = false;

//==========================================================================//
//                        14. EXPERT WAYPOINTS AND ALIGNMENT                //
//==========================================================================//

float puck_pose_x = 0.0f;
float puck_pose_y = 0.0f;

float puck_rear_x = 0.0f;
float puck_rear_y = 0.0f;

float retreat_start_x = 0.0f;
float retreat_start_y = 0.0f;

float puck_extra_x = 0.0f;
float puck_extra_y = 0.0f;

const float RET_DIST = 80.0f; // retreatng distance after detecting magnet
const float RET_DMD = 0.5f; // reverse demand for retretaing behaviour

const float whisker_rad = 150.0f; // robots whisker diameter (130 mm measures, 150 mm for added security)

bool ret_active = false; // True while reversing takes place
bool align_active = false; // True once alignment sequence is started
bool extra_wp_needed = false; // True if robot needs additonal way point
bool turn_home_needed = false; // True after arriving at point behind puck (turns to face home without early heading alignment trigger). 

// - Expert Travel States -
enum TurnMode {TURN_NORM, TURN_EXTRA, TURN_REAR, TURN_HOME};
enum TravelMode {TRAV_NORM, TRAV_EXTRA, TRAV_REAR, TRAV_HOME};

TurnMode turn_mode = TURN_NORM;
TravelMode travel_mode = TRAV_NORM;

//==========================================================================//
//                        15. TIMER SETTINGS                                //
//==========================================================================//

const unsigned long TOTAL_TIME_MS = 240000;

unsigned long startTime;
unsigned long timeUpdate = 0;

bool timer_on = false;
bool time_up = false; // True when timer xpires

//==========================================================================//
//==========================================================================//
//                        SECTION B: FUNCTIONS                              //
//==========================================================================//
//==========================================================================//

//==========================================================================//
//                        1. SPEED MEASUREMENT                              //
//==========================================================================//
// -------------------- SPEED MEASUREMENT FUNCTION --------------//

// Updating wheel speed using encoder measurements.
// elapsed time is the amount of time since last call in ms

void updateWheelSpeeds(unsigned long elapsed_time) {

  // difference in counts since last estimate
  long count_difference = count_e0 - last_e0;
  long count_difference1 = count_e1 - last_e1;

  // update "last" count for next time
  last_e0 = count_e0;
  last_e1 = count_e1;

  // compute speed (counts per ms)
  speed_e0 = (float)count_difference / (float)elapsed_time;
  speed_e1 = (float)count_difference1 / (float)elapsed_time;

  // Low pass filter:
  speed_e0_filt = (ALPHA * speed_e0) + ((1.0 - ALPHA) * speed_e0_filt);
  speed_e1_filt = (ALPHA * speed_e1) + ((1.0 - ALPHA) * speed_e1_filt);

  // reset timestamp for next interval
  speed_ts = millis();
}

//==========================================================================//
//                        2. CALIBRATION                                    //
//==========================================================================//

// -------------------------------------------------------------------------//
// startCal() - for beginning calbration spin and restting cal state.
// UpdateCal() - Called every loop during CALIBRATE state until calibration takes place.
// coodown() - A puse to realign the robot. 
// -------------------------------------------------------------------------// 
void startCal() {

  // Resetting magnetic max/min;
  for (int n = 0; n < NUM_AXIS; n++) {

    maximum[n] = -9999.9;
    minimum[n] = +9999.9;
  }

  mag_calibrated = false;

  // Reset line sensing max/min
  for (int n = 0; n < NUM_SENSORS; n++) {
    line_sensors.maximum[n] = 0.0f;
    // max set low
    line_sensors.minimum[n] = 1023.0f; // min set high
    //line_sensors.scaling[n] = 1.0f;
  }

  line_calibrated = false;

  // time flags updated to keep calibration time limited 
  cal_start = millis();
  cal_PID_ts = millis();

  left_pid.reset();
  right_pid.reset();

  // Start Timer for countdown from the point of robot movement and initiate the LED display
  startTime = millis();
  timeUpdate = millis();
  timer_on = true;
  time_up = false;

  state = CALIBRATE;
  Serial.println("CALIBRATION START");
}

// Update the Calibration spin, the sensors and when time finsihed, calculate the calibration constants 
// and initiate the cooldown post calibration

void UpdateCal() {

  // PID spin to minimise drift

  if (millis() - cal_PID_ts >= PID_MS) {
    cal_PID_ts += PID_MS;
    float left_pwm = left_pid.update(+CAL_DMD, speed_e1_filt);
    float right_pwm = right_pid.update( -CAL_DMD, speed_e0_filt);

    motors.setPWM((int)left_pwm, (int)right_pwm);
  }

  // Update Magnetic max/min
  magnetometer.getReadings();
  for (int n = 0; n < NUM_AXIS; n++) {

    if (magnetometer.readings[n] > maximum[n]) {
      maximum[n] = magnetometer.readings[n];
    }

    if (magnetometer.readings[n] < minimum[n]) {
      minimum[n] = magnetometer.readings[n];
    }
  }

  // Update Line Sensor min/max
  line_sensors.readSensorsADC();
  for (int n = 0; n < NUM_SENSORS; n++) {
    float r = line_sensors.readings[n];
    if (r > line_sensors.maximum[n]) {
      line_sensors.maximum[n] = r;
    }
    if (r < line_sensors.minimum[n]) {
      line_sensors.minimum[n] = r;
    }
  }
  // Finish spinning once calibration is done
  if (millis() - cal_start >= CAL_MS) {

    motors.setPWM(0, 0);
    for (int n = 0; n < NUM_AXIS; n++) {
      // Final stored and scaled values:

      mag_range[n] = (maximum[n] - minimum[n]);
      float half_range = mag_range[n] * 0.5f;
      mag_offset[n] = (minimum[n] + half_range);
      if (half_range < 1.0f) {
        half_range = 1.0f;
      }
      mag_scaling[n] = 1 / (half_range);
    }

    mag_calibrated = true;
    for (int sensors = 0; sensors < NUM_SENSORS; sensors++) {
      line_range[sensors] = line_sensors.maximum[sensors] - line_sensors.minimum[sensors];
      line_sensors.scaling[sensors] = line_range[sensors];
    }
    line_calibrated = true;

    Serial.println("CALIBRATION COMPLETE");

    // - MAG REALIGNMENT: CAPTURE MAG REF - //
    calcCalibratedMag();
    initial_mag_heading = atan2(magcalibrated[1], magcalibrated[0]);
    Serial.print("Ref Heading: ");
    Serial.println(initial_mag_heading);
    // -----------------------------------------
    Serial.print("Mag_calibrated:");
    Serial.print(mag_calibrated);
    Serial.print("Line_calibrated:");
    Serial.print(line_calibrated);

    motors.setPWM(0, 0);
    cooldown_start = millis();
    state = COOLDOWN;
  }
}

// Cooldown after calibration to prevent rapid movements
void cooldown() {
  if (millis() - cooldown_start >= CLDWN_MS) {

    left_pid.reset();
    right_pid.reset();
    turn.reset();
    heading.reset();

    cal_realign = true;
    setTurn(initial_mag_heading);
    //test_start = false;
    //stateDebugTest();
  }
}

//==========================================================================//
//                        3. MAGNETOMETER + LINE SENSOR HELPER              //
//==========================================================================//

// - FUNCTIONS FOR READINGS - //

// From the magnetometer readings, obtains the normalised values
void calcCalibratedMag() {
  magnetometer.getReadings();
  for (int n = 0; n < NUM_AXIS; n++) {
    magcalibrated[n] = (magnetometer.readings[n] - mag_offset[n]) * mag_scaling[n];
  }
}

// Calculates the magnitude of calibrated magnetic vectors
float calcMagMagnitude() {
  float x = magcalibrated[0];
  float y = magcalibrated[1];
  float z = magcalibrated[2];
  return sqrt(x * x + y * y + z * z);
}

// - PUCK DETECTION - //
// Using the magnetic magnitude threshold and a holing time, confirm puck detection.
// The hold time shoudl stop it from stopping due to any random spike. 
bool detectPUCK() {
  if (mag_calibrated == false) {
    return false;
  }

  calcCalibratedMag();
  float m = calcMagMagnitude();

  if (m > MAG_THRESHOLD) {
    if (mag_start == 0) {
      mag_start = millis();
    }
    if (millis() - mag_start >= MAG_HOLD_MS ) {
      return true;
    }
    return false;
  }
  mag_start = 0;
  return false;
}

// -- LINE DETECTION -- //
// Detect the outer borders comparing to the reference threshold
bool lineDetected() {
  if (line_calibrated == false) {
    return false;
  }
  line_sensors.calcCalibratedADC();
  for (int n = 0; n < NUM_SENSORS; n++) {
    if (line_sensors.calibrated[n] > LINE_THRESHOLD) {
      return true;
    }
  }
  return false;
}

// -- FUNCTION FOR MAGNETIC DEBUG --//
// Raw magnetometer data wihtout calibration - Use to check if magnetometer fucntions as expeted
void debugMagRawDiff() {
  static unsigned long t = 0;
  if (millis() - t < 200)
    return; t = millis();
  magnetometer.getReadings();
  Serial.print("RAW: ");
  Serial.print(magnetometer.readings[0]);
  Serial.print(",");
  Serial.print(magnetometer.readings[1]);
  Serial.print(",");
  Serial.print(magnetometer.readings[2]);
  Serial.println();
}
// Normalised magnetometer data 
void debugMag() {
  static unsigned long updte_ts = 0;
  if (millis() - updte_ts >=  DEBUG_MS) {
    updte_ts = millis();
    if (mag_calibrated == false) {
      Serial.println("Not CALIBRATED YET");
      return;
    }

    calcCalibratedMag();
    Serial.print(magcalibrated[0], 3);
    Serial.print(",");
    Serial.print(magcalibrated[1], 3);
    Serial.print(",");
    Serial.print(magcalibrated[2], 3);
    float m = calcMagMagnitude();
    Serial.print("Magnitude: ");
    Serial.print(m, 3);
    Serial.print("Threshold: ");
    Serial.println(MAG_THRESHOLD, 3);
  }
}


// -- LINE SENSOR DEBUG -- //

void debugLine() {
  static unsigned long updte_ts = 0;
  if (millis() - updte_ts >=  DEBUG_MS) {
    updte_ts = millis();
    if (line_calibrated == false) {
      Serial.println("Not CALIBRATED YET");
      return;
    }

    line_sensors.calcCalibratedADC();
    Serial.print("Line Calibration Reading:");
    for (int n = 0; n < NUM_SENSORS; n++) {
      Serial.print(line_sensors.calibrated[n], 3);
      if (n < NUM_SENSORS - 1) {
        Serial.print(",");
      }
    }

    Serial.println();

  }
}


//==========================================================================//
//                        4. TURN CONTROL                                   //
//==========================================================================//

// ----------------- Angle difference calculator --------------- //

float angleDiff(float target_angle) {
  float diff = atan2(sin(target_angle - pose.theta), cos(target_angle - pose.theta));
  return diff;
}

// --------------------- Travel Heading calculator ------------- //
float headingToXY(float x, float y) {
  float dx = x - pose.x;
  float dy = y - pose.y;
  return atan2f(dy, dx);
  //Serial.println(atan2f(dy,dx));
}

// --------------------- SET TURN FUNCTIONS --------------------- //
// Set Turn begins turns based on the angel it recieves, 
// setTurnToXY identifies the travel heading and calls set Turn. 
void setTurn(float target) {
  start_theta = pose.theta;
  target_angle = target;
  turning = true;

  left_pid.reset();
  right_pid.reset();
  turn.reset();

  state = TURN_TO;
}

void setTurnToXY(float x, float y, TurnMode mode) {
  turn_mode = mode;
  float desired_angle = headingToXY(x, y);
  setTurn(desired_angle);
}

// --------------------- CHECK TURN FUNCTION ------------------- //
// Turn controller, keeps track of whether turns are happening and should return true during turns.
// Motor stops after turns commented to restore momentum after turns and reduce lag.
bool checkTurnPID() {

  float error = angleDiff(target_angle);
  float e = fabs(error);
  // Stop if the target angle is reached
  if (e < TURN_TOL_RAD) {
    //motors.setPWM(0, 0);
    turning = false;
    pose.theta = target_angle;
    left_pid.reset();
    right_pid.reset();
    return false;
  }

  float turn_speed = turn.update(0.0, error);
  if (fabs(turn_speed) < TURN_MIN) {
    if (turn_speed > 0) {
      turn_speed = TURN_MIN;
    }
    else {
      turn_speed = -TURN_MIN;
    }
  }

  if (turn_speed > TURN_MAX) {
    turn_speed = TURN_MAX;
  } else if (turn_speed < -TURN_MAX) {
    turn_speed = -TURN_MAX;
  }

  float left_demand = turn_speed;
  float right_demand = -turn_speed;

  float  left_pwm = left_pid.update(left_demand, speed_e1_filt);
  float  right_pwm = right_pid.update(right_demand, speed_e0_filt);

  motors.setPWM((int)left_pwm, (int)right_pwm);
  return true;
}

// --------------------- DEBUG TURN FUNCTION ------------------- //

void debugTurn() {
  static unsigned long debug_ts = 0;
  if (millis() - debug_ts < DEBUG_MS) {
    return;
  }
  debug_ts = millis();
  float turn_err = angleDiff(target_angle);
  Serial.print("Turn error");
  Serial.println(turn_err * 180.0f / PI, 2);
  Serial.print(",");
  Serial.print("Current Angle");
  Serial.print(pose.theta * 180.0f / PI, 1);
}

//==========================================================================//
//                        4. TRAVEL CONTROL                                 //
//==========================================================================//
// -------------------- Distance ot target calculator ---------- //

float distToTarget(float x, float y) {
  float dx = x - pose.x;
  float dy = y - pose.y;
  return sqrt(dx * dx + dy * dy);
}

// ---------------------- SET TRAVEL FUNCTION ------------------ //

void setTravel( float x, float y) {
  target_x = x;
  target_y = y;
  travelling = true;
  left_pid.reset();
  right_pid.reset();
  heading.reset();
  speed_e0_filt = 0.0;
  speed_e1_filt = 0.0;
  state = TRAVEL_TO;
}

// ---------------------- CHECK TRAVEL FUNCTION ----------------- //

bool checkTravel() {

  if (!travelling) {
    return;
  }

  float dx = target_x - pose.x;
  float dy = target_y - pose.y;
  float distance = sqrt(dx * dx + dy * dy);

  if (distance <= TRAVEL_TOL) {
    //motors.setPWM(0, 0);
    travelling = false;
    left_pid.reset();
    right_pid.reset();
    heading.reset();
    return;
  }

  desired_heading = atan2f(dy, dx);
  theta_L = angleDiff(desired_heading);

  float fwd = TRAVEL_SPD;
   //Braking to reduce jolts - remove if behaviour does not improve 
    if (distance < 50) {
      fwd = 0.3;
    }
  //  if (distance < 50) {
  //    fwd = 0.5;
  //  }

  turn_ctrl = heading.update(0.0, theta_L);
  turn_ctrl = 2.0 * turn_ctrl;

  // remove if weird behaviour returns
  //  if (turn > HEADING_TURN_MAX){
  //    turn = HEADING_TURN_MAX;
  //  }
  //  if (turn < -HEADING_TURN_MAX){
  //    turn = -HEADING_TURN_MAX;
  //  }

  float left = fwd + turn_ctrl;
  float right = fwd - turn_ctrl;
  float left_pwm  = left_pid.update(left,  speed_e1_filt);
  float right_pwm = right_pid.update(right, speed_e0_filt);

  motors.setPWM((int)left_pwm, (int)right_pwm);
  return true;
}

// ----------------------- DEBUG TRAVEL ----------------------------//

void debugTrav() {
  static unsigned long debug_ts = 0;
  if (millis() - debug_ts < DEBUG_MS) {
    return;
  }

  debug_ts = millis();

  float x = pose.x;
  float y = pose.y;

  float dx = target_x - pose.x;
  float dy = target_y - pose.y;
  float dist = sqrt(dx * dx + dy * dy);

  float des_head = atan2f(dy, dx);
  float head_err = angleDiff(des_head);
  Serial.print("STATE = ");
  Serial.print(state);
  Serial.print("Stage = ");
  Serial.print(route_stage);
  Serial.print("Target ");
  Serial.print(current_target);

  Serial.print("Pose ");
  Serial.print(pose.x, 1);
  Serial.print(",");
  Serial.print(pose.y, 1);

  Serial.print("Target: ");
  Serial.print(target_x, 1);
  Serial.print(",");
  Serial.print(target_y, 1);

  Serial.print("Heading error: ");
  Serial.print(head_err * 180 / PI, 2);

  Serial.print("DISTANCE: ");
  Serial.println(dist, 1);
}


//==========================================================================//
//                        5. WAYPOINTONTROL                                 //
//==========================================================================//
// Start next route in the route pathway, find the next WP anf turn to face it
void startRoute() {
  current_target = route_order[route_stage];
  target_x = target_x_num[current_target];
  target_y = target_y_num[current_target];
  setTurnToXY(target_x, target_y, TURN_NORM);

}

// Adnace the route stage array to the next wp in the search
void nextWP() {
  //motors.setPWM(0, 0);
  route_stage++;
  if (route_stage >= route_length) {
    route_stage = 0;
  }
  state = ROUTE_START;
}

//==========================================================================//
//                        6. ROBOT REACTIONS FOR PUCK AND LINES             //
//==========================================================================//

// ------------------------- PUSH PUCK ----------------------------------- //
// unused for expert level
void PuckStop() {
  if (mag_found) {
    return;
  }
  if (detectPUCK() == true) {
    mag_found = true;
    motors.setPWM(0, 0);

    travelling = false;
    turning = false;

    left_pid.reset();
    right_pid.reset();
    turn.reset();
    heading.reset();

    Serial.println("PUCK DETECTED. PUSH PUCK");
    state = PUSH_PUCK;
  }
}

// unsued for expert level 
void PushPuck() {
  if (lineDetected()) {
    motors.setPWM(0, 0);
    Serial.println("LINE DETECTED. STOP");

    mag_found = false;
    mag_start = 0;

    state = STOP_FOUND;
    return;
  }

  motors.setPWM(30, 30);
}

//==========================================================================//
//                        7. TIMER                                          //
//==========================================================================//

void updateTimer() {

  if (time_up == true) {
    display.clear();
    display.gotoXY(0, 1);
    display.print("DONE");
    return;
  }

  if (timer_on == false) {
    return;
  }

  if (millis() - timeUpdate <= 1000) return;
  timeUpdate = millis();

  unsigned long time_elapsed_s = (millis() - startTime);
  long time_remaining = TOTAL_TIME_MS - time_elapsed_s;

  // update eeveyr second
  if (time_remaining <= 0) {
    time_remaining = 0;
    timer_on = false;
  }

  int minutes = time_remaining / 60000;
  int seconds = (time_remaining % 60000) / 1000;

  display.clear();
  display.gotoXY(0, 1);

  display.print(minutes);
  display.print(":");

  if (seconds < 10) display.print("0");
  display.print(seconds);

  //Time up
  if (time_remaining == 0) {
    time_up = true;
    timer_on = false;
  }

}

// Hard stop when time is up
void StopHunt() {
  timer_on = false;
  time_up = true;

  motors.setPWM(0, 0);
  travelling = false;
  turning = false;

  left_pid.reset();
  right_pid.reset();
  turn.reset();
  heading.reset();

  state = STOP;
  //setTurnToXY(Home_x, Home_y);

}

//==========================================================================//
//                        8. PUCK RETURN HELPERS                            //
//==========================================================================//


// Get Puck coordinates from its current route target points
void PuckCoordinates() {
  puck_pose_x = target_x_num[current_target];
  puck_pose_y = target_y_num[current_target];
}

// Retreat helper - reverses until we hit our target distance - a safe  8 cm for now. 
void reversePID(float REV_DMD) {

  float left_pwm = left_pid.update(-REV_DMD, speed_e1_filt);
  float right_pwm = right_pid.update(-REV_DMD, speed_e0_filt);

  motors.setPWM((int)left_pwm, (int)right_pwm);
}

// ====================================================================================== //
// Get the point behind the puck the robot needs to go to, along the vector towards home. // 
// The Push direction is the unit vector form the puck to the home coordinates.           //
// Then the rear cordinate can just be found by using the porduct of the unit vector and  //
// whisker radiues to find the distance of the rear point from the puck point. Then fin   //
// relative position from the puck location.                                              //
// ====================================================================================== // 
void GetRearPoint() {

  // x and y vector values from home
  float vx = Home_x - puck_pose_x;
  float vy = Home_y - puck_pose_y;

  float k = sqrt(vx * vx + vy * vy);

  //unit vector
  float push_x = vx / k;
  float push_y = vy / k;
  
  // Point behind the puck robot needs to go to
  puck_rear_x = puck_pose_x - whisker_rad * push_x;
  puck_rear_y = puck_pose_y - whisker_rad * push_y;
}
// ===================================================================================//
// Some locations need additonal waypoints to go around the puck and not run into it. // 
// Based on Lasbheet 4, we find a lateral offset, left or right, to go aorund the puck//
// depending on which side the robot is currently, relative to the puck.              //
// So we found the unit vector for the push direction, then found the perpedincular   //
// left/right unit vectors and the using a corss porduct the side to move around the  //
// puck is found.                                                                     //
// ===================================================================================//
void GetXtraPoint(int push_side = 0) {
  // Push vector
  float push_x = Home_x - puck_pose_x;
  float push_y = Home_y - puck_pose_y;

  // normaliser
  float k = sqrt(push_x * push_x + push_y * push_y);

  // unit vector calc
  push_x = push_x / k;
  push_y = push_y / k;

  // Perpendicular lft and right coordinates
  float perp_right_x = push_y;
  float perp_right_y = -push_x;
  float perp_left_x = -push_y;
  float perp_left_y = push_x;

  // The vector from the puck to the robot 
  float v_x = pose.x - puck_pose_x;
  float v_y = pose.y - puck_pose_y;
  // cross product of the vectors to identify sides 
  float cross = push_x * v_y - push_y * v_x;

  float side_x;
  float side_y;

  // picking the side 
  if (cross > 0) {
    side_x = perp_left_x;
    side_y = perp_left_y;
  } else {
    side_x = perp_right_x;
    side_y = perp_right_y;
  }

  puck_extra_x = puck_pose_x + whisker_rad * side_x;
  puck_extra_y = puck_pose_y + whisker_rad * side_y;
}

// Boolean to decide if the robot is already behind puck relative to the push direction // 
bool behindPuck() {
  // Push vector
  float push_x = Home_x - puck_pose_x;
  float push_y = Home_y - puck_pose_y;

  // Vector normalisation
  float k = sqrt(push_x * push_x + push_y * push_y);
  push_x = push_x / k;
  push_y = push_y / k;
  // Puck to robot vector
  float v_x = pose.x - puck_pose_x;
  float v_y = pose.y - puck_pose_y;

  // Dot product to locate if robot s behind puck
  float dot = v_x * push_x + v_y * push_y;

  return (dot < -0.2f);

}

void PlanAlignment() {
  PuckCoordinates();
  GetRearPoint();

  if (behindPuck()) {
    extra_wp_needed = false;
  } else {
    extra_wp_needed = true;
    GetXtraPoint();
  }
}
//==========================================================================//
//==========================================================================//
//                        SECTION C: STATE CONTROL FUNCTIONS                //
//==========================================================================//
//==========================================================================//

// 1. ---------------------- CALIIBRATION STATE --------------------------- //
void stateCal() {
  UpdateCal();
  // Calls a cooldown when done with calibration
}

// 2. ----------------------- COOL DOWN STATE ----------------------------- //
void stateCLDWN() {
  cooldown();
  // Moves to realign robot to 0 deg once done
}

// 3. - ROUTE START - //
void stateROUTESTART() {
  if (route_started == false) {
    route_started = true;
    route_stage = 0;
  }
  startRoute();               // STarts the route between waypoints
}

// 4. --------------------------- TURN STATE ----------------------------- //
// Basic idea - chckTurnPID() should return false when done. then should move to
// appropriate states afterwrads. 
// Post Turn Tasks: 
// 1. cal_realign - Realigns after calibration when done.
// 2. TURN_EXTRA - Turns to extra way point to the side of puck location. 
// 3. TURN_REAR - Travels tp point behind puck. 
// 4. turn_home_needed - Go directly to PUSH_HOME state
// 5. TURN_HOME - Travel to home after achieveing target heading turn. 
// 6. TEST_TURN - stop afteer for debugging. 
// 7. Normal behaviour - travel to next waypoint. 
// ------------------------------------------------------------------------ // 
void stateTurnTo() {

  // Print turn data for tuning
  if (debug_mode == true) {
    debugTurn();
    //motors.setPWM(0, 0);
  }

  // Keep turns until we reach target
  if (turning == true) {
    if (checkTurnPID()){
    return;
    }
  }
  if (cal_realign == true) {
    cal_realign = false;
    test_start = false;
    stateDebugTest();
    return;
  }
  if (turn_mode == TURN_EXTRA) {
    setTravel(puck_extra_x, puck_extra_y);
    return;
  }
  if (turn_mode == TURN_REAR) {
    setTravel(puck_rear_x, puck_rear_y);
    return;
  }

  if (turn_home_needed == true && turning == false){
    turn_home_needed = false;
    left_pid.reset();
    right_pid.reset();
    heading.reset();
    turn.reset();
    state = PUSH_HOME;
    return;
  }
  
  if (turn_mode == TURN_HOME || go_home) {
    // If returning home then we return to origin
    setTravel(Home_x, Home_y);
    return;
  }

  if (turning == false && test_running && test_mode == TEST_TURN) {
    state = STOP;
    return;
  }

  setTravel(target_x, target_y);
}



// 5. ------------------------------- TRAVEL STATE ----------------------- //
// Basic idea - chckTravel() should return false when done. then should move to
// appropriate states afterwrads. 
// Post Turn Tasks: 
// 1. TRAV_EXTRA - Turn to extra way point to the side of puck location.
// 2. TRAV_REAR - Turns to face home
// 3. TRAVE_HOME - Go to HOME_RETURN state.
// 7. Normal behaviour - next waypoint. 
// ------------------------------------------------------------------------ // 
void stateTravelTo() {

  if (debug_mode == true) {
    debugTrav();
    //motors.setPWM(0, 0);

  }

  if (travelling == true) {
    if (checkTravel()){;
    return;
    }
  }

  if (travel_mode == TRAV_EXTRA) {
    travel_mode = TRAV_REAR;
    setTurnToXY(puck_rear_x, puck_rear_y, TURN_REAR);
    return;
  }

//    if (travel_mode == TRAV_REAR) {
//    travel_mode = TRAV_NORM;
//    left_pid.reset();
//    right_pid.reset();
//    heading.reset();
//    turn.reset();
//    state = PUSH_HOME;
//    return;
//  }
  
  if (travel_mode == TRAV_REAR) {
    travel_mode = TRAV_NORM;
    float angle_to_puck = headingToXY(puck_pose_x, puck_pose_y);
    turn_home_needed = true;
    setTurnToXY(Home_x, Home_y, TURN_HOME);    
    return;
  }  

  if (travel_mode == TRAV_HOME || go_home) {
    home_wait_start = millis();
    state = HOME_RETURN;
    return;
  }

  if (travelling == false && test_running == true && test_mode == TEST_TRAVEL) {
    state = STOP;
    return;
  }
  state = NEXT_WP;
}

// 5. ------------------------------ Next Waypoint --------------------------------- //

void stateNextWP() {
  nextWP();
}

// 6. -------------------------------- PUSH PUCK ----------------------------------- //
void statePUSHPUCK() {
  PushPuck();
  // Goes to Ste of STOP_FOUND when we hit a line
}

// 7. -------------------------------Stop Found state ------------------------------ //
void stateStopFound() {
  motors.setPWM(0, 0);
  digitalWrite(LED_PIN, HIGH);

  // We now need to go home since the line has been found
  if (go_home == false) {
    go_home = true;
    mag_found = false;
    mag_start = 0;

    setTurnToXY(Home_x, Home_y, TURN_HOME);
  }
}

// 8. -------------------------------- Home Waiting state ------------------------- //
// --- MAG THETA RESET APPLIED HERE ---
void stateHomeReturn() {
  //Stop and reset motor and led
  motors.setPWM(0, 0);
  digitalWrite(LED_PIN, LOW);

  if (millis() - home_wait_start < HOME_WAIT_MS) {
    return;
  }
  //RESET EVERYTHNG
  route_stage = 0;
  route_started = false;
  current_target = 0;
  go_home = false;

  ret_active = false;
  align_active = false;
  extra_wp_needed = false;

  mag_start = 0;
  mag_found = false; 

  turn_home_needed = false;
  turn_mode = TURN_NORM;
  travel_mode = TRAV_NORM;

  left_pid.reset();
  right_pid.reset();
  turn.reset();
  heading.reset();

  if (time_up) {
    state = STOP;
    return;
  }
  state = ROUTE_START;
}

// 9. --------------------------------------- Stop State ------------------------- //
void stateStop() {
  motors.setPWM(0, 0);

  if (debug_line == true) {
    debugLine();
    return;
  }
  if (debug_mag == true) {
    debugMag();
    return;
  }
  if (test_mode == TEST_TURN) {
    debugTurn();
  }
  if (test_mode == TEST_TRAVEL) {
    debugTrav();
  }
}

// 9. --------------------------------------- Debug State ------------------------- //
void stateDebugTest() {

  if (test_start) {
    return;
  }

  test_start = true;
  test_running = (test_mode != TEST_ROUTE);
  if (test_mode == TEST_TURN) {
    setTurn(turn_test);
    return;
  } else if ( test_mode == TEST_TRAVEL) {
    target_x = test_x;
    target_y = test_y;
    setTurnToXY(target_x, target_y, TURN_NORM);
    return;
  } else {
    state = ROUTE_START;
  }

}

// 9. --------------------------------------- Retreat State ------------------------- //
void stateRetreat() {
  // reverse 8 cm to a saf point away from puck to not hit it. Tune mag_threshold and hold time to also prevent bumps
  if (ret_active == false) {
    ret_active = true;
    retreat_start_x = pose.x;
    retreat_start_y = pose.y;

    left_pid.reset();
    right_pid.reset();
  }
  float dx = pose.x - retreat_start_x;
  float dy = pose.y - retreat_start_y;

  float dist = sqrt(dx * dx + dy * dy);

  reversePID(RET_DMD);

  // If reversing time is done, stop and move to next alignment state
  if (dist >= RET_DIST) {
    motors.setPWM(0, 0);
    //retreat_ts = 0;

    mag_start = 0;
    mag_found = false;
    ret_active = false;   

    if (extra_wp_needed == true) {
      travel_mode = TRAV_EXTRA;
      setTurnToXY(puck_extra_x, puck_extra_y, TURN_EXTRA);
    } else {
      travel_mode = TRAV_REAR;
      setTurnToXY(puck_rear_x, puck_rear_y, TURN_REAR);
    }
    return;
  }
}


// 10. --------------------------------------Bring Puck home State ------------------------- //
void statePUSHHOME() {
  
  float dist_home = distToTarget(Home_x, Home_y);
  if (dist_home <= (TRAVEL_TOL + 2.0f)){
    motors.setPWM(0,0);
    mag_found = false;
    mag_start = 0;
    home_wait_start = millis();
    state = HOME_RETURN;
    return;
  }

  float dx = Home_x - pose.x;
  float dy = Home_y - pose.y;
  float push_head = atan2f(dy,dx);
  float head_diff = angleDiff(push_head);
  float head_corr = 1.5f  * heading.update(0.0f, head_diff);

  float fwd = TRAVEL_SPD;
  float left_pwm = left_pid.update(fwd + head_corr, speed_e1_filt);
  float right_pwm = right_pid.update(fwd - head_corr, speed_e0_filt);
  motors.setPWM((int)left_pwm, (int)right_pwm);
  
}


//==========================================================================//
//==========================================================================//
//                        SECTION C: SETUP                                  //
//==========================================================================//
//==========================================================================//

// ---------------------------- SETUP FUNCTION ---------------------------- //

void setup() {
  // Serial for debugging.
  // ------------------------

  Serial.begin(9600);

  pinMode(BUZZER_PIN, OUTPUT);
  analogWrite(BUZZER_PIN, 0);
  display.init();
  display.clear();

  Wire.begin();
  // Initialising Motor and encoder hardware ------
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  // Pose Initialisation

  pose.initialise(0, 0, 0);

  // Line sensor initialise

  line_sensors.initialiseForADC();
  // Magentometer initialise check
  if (!magnetometer.initialise()) {
    Serial.println("MAG INIT FAIL");
    while (1);
  }
  magnetometer.mag.enableDefault();

  // update "last" count for next time -------------
  last_e0 = count_e0;
  last_e1 = count_e1;

  //Initialising Speed measurements to 0 ------------
  speed_e0 = 0.0;
  speed_e1 = 0.0;
  speed_e0_filt = 0.0;
  speed_e1_filt = 0.0;

  // Timing initialisation -------------------------
  speed_ts = millis();
  pose_ts = millis();
  pid_ts = millis();

  // Initialise all gains --------------------------
  left_pid.initialise( 60, 0.02, 0.001 );
  right_pid.initialise( 60, 0.02, 0.001 );
  turn.initialise(2.5, 0.0, 0.02);
  heading.initialise(2.0, 0.0, 0.0);
  // Remember to reset PID -------------------------
  // used any delay()

  left_pid.reset();
  right_pid.reset();
  turn.reset();
  heading.reset();

  debug_mode = false;
  debug_mag = false;
  debug_line = false;
  test_mode = TEST_ROUTE;

  startCal();
  // TEST COMMAND -----------------------------------
  Serial.println("TEST 1: CALIBRATION");
}


//==========================================================================//
//==========================================================================//
//                        SECTION D: LOOP                                   //
//==========================================================================//
//==========================================================================//

void loop() {

  // TIMER:
  updateTimer();

  if (time_up) {
    StopHunt();
    return;
  }

  //------------------- POSE UPDATE -------------------//
  static unsigned long last_update_ms = 0;
  unsigned long Update = millis();
  if (Update - last_update_ms >= POSE_MS) {
    last_update_ms += POSE_MS;
    pose.update();
  }

  // ----------- Wheel Speed Update Section -----------//
  unsigned long elapsed_time = millis() - speed_ts;
  if (elapsed_time >= SPEED_MS) {
    updateWheelSpeeds(elapsed_time);
  }

  if (go_home == false && align_active == false && state == TRAVEL_TO && detectPUCK()) {

    motors.setPWM(0, 0);
    travelling = false;
    turning = false;

    left_pid.reset();
    right_pid.reset();
    turn.reset();
    heading.reset();

    PlanAlignment();

    turn_mode = TURN_NORM;
    travel_mode = TRAV_NORM;
    align_active = true;
    ret_active = false;

    state = RETREAT;
  }

  switch (state) {
    case CALIBRATE:
      stateCal();
      break;

    case COOLDOWN:
      cooldown();
      break;
      
    case ROUTE_START: {
        stateROUTESTART();
        break;
      }

    case TURN_TO: {
        stateTurnTo();
        break;
      }

    case TRAVEL_TO: {
        stateTravelTo();
        break;
      }

    case NEXT_WP: {
        stateNextWP();
        break;
      }

    case PUSH_PUCK: {
        statePUSHPUCK();
        break;
      }

    case STOP_FOUND: {
        stateStopFound();
        break;
      }

    case HOME_RETURN: {
        stateHomeReturn();
        break;
      }

    case RETREAT: {
        stateRetreat();
        break;
      }

    case PUSH_HOME: {
        statePUSHHOME();
        break;
      }
    case STOP: {
        stateStop();
        break;
      }
  }
}
