// Commented Code used by DV22277 - Michael Linder for Expert task, achieving reliably 7+ pucks in 4 minutes

// Inclusion of .h files
#include "Motors.h"
#include "PID.h"
#include "LineSensors.h"
#include "Magnetometer.h"
#include "Kinematics.h"
#include "Encoders.h"
#include "oled.h"
#include <Wire.h>           // I2C Protocol to interact with Magnetometer
#include <LIS3MDL.h>

// Declaration of class instances
Kinematics_c pose;          // Internal Position tracking
Motors_c motors;            // Send signals to motors
PID_c left_pid;             // To control the left motor
PID_c right_pid;            // To control the right motor
PID_c left_pid_rev;         // To control the left motor
PID_c right_pid_rev;        // To control the right motor
PID_c heading;              // To control turning operations
LineSensors_c line_sensors; // Black/White sensing
LIS3MDL mag;                // Magntometer
OLED_c display(1, 30, 0, 17, 13); // Pins: clk, mos, res, dc, cs

// Timestamps and Timings
unsigned long speed_est_ts; // timestamp for speed estimation - used for pose updates also
#define SPEED_EST_MS 10     // 10ms
unsigned long home_ts;      // timestamp for pausing at home
# define Pause_MS 4000      // 4s as required by assessment 
unsigned long oled_ts = 0;  // timestamp to update the oled during debugging
#define OLED_UPDATE_MS 500  // 500ms to not flood the oled 
unsigned long beep_stop_time = 0; // timestampe for beeps

// Main states for the foraging machine
enum State_t {
  PATROL,
  FOUND,
  STOPPED,
  GOHOME,
  PUSHHOME,
  DEBUG,
  END
};
State_t currentState = DEBUG;

// States for a GOTOXY function - used to track internally what stage in the movement it is at
enum TravelState {
  TRAVEL_IDLE,
  TRAVEL_ROTATE,
  TRAVEL_FORWARD,
  TRAVEL_DONE,
  TRAVEL_ERROR
};
TravelState travel_state = TRAVEL_IDLE;

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


// Turning and fwd flags to stop reinitialising turns and travelling if already occuring
bool Turn_status  = false;
bool travel_status = false;


// Global Targets for movement operations
float Target_angle;
float goal_x;
float goal_y;

// Tolerances for Movement Operations
#define DIST_TOLERANCE 3.0   // mm
#define TURN_TOLERANCE 0.01 // rad


// Magnetometer Params
#define NUM_COORD 3       // X,Y,Z
#define M_THRESHOLD 4.2   // Adjust based on calibration dependant on surroundings 4.2 Used for Home desk tests due to metal legs and drawer slides
float M_min[NUM_COORD], M_max[NUM_COORD], M_range[NUM_COORD], M_offset[NUM_COORD], M_scaling[NUM_COORD];
float m_calibrated[NUM_COORD], m_val;
unsigned long M_read_ts;


// Definite Buzzer Pin
#define BUZZER_PIN 6

// Waypoints of the puck positions - measured from at home map to centre of 'BMW' Calibration circle
#define NUM_WAYPOINTS 7   //0, 1,   2,     3,     4,     5,     6    
float x[ NUM_WAYPOINTS ] = {0, 61,  248.5, 416.5, 398.5, 343.0, 156.5};
float y[ NUM_WAYPOINTS ] = {0, 266, 214,   268.5, 116.5, -9.0,  84.0 };
int waypoint_index;

// Spin Test Variables - Used In Debugging only
bool test_started = false;

// Setup Runs once
void setup() {
  // Start Up Serial and allow time to connect to laptop for debugging
  Serial.begin(9600);
  delay(2000);

  Wire.begin();                 // Init I2C
  pinMode(BUZZER_PIN, OUTPUT);  // Init Buzzer Pin
  analogWrite(BUZZER_PIN, 0);   // Ensure it starts quiet

  // Zero Counts and last Speeds
  last_e0 = count_e0;
  last_e1 = count_e1;
  last_speed_e0 = 0;
  last_speed_e1 = 0;

  // Initialise PID Controllers
  left_pid.initialise( 15.0, 0.1, 0.0); // Left motor does start slightly higher pwm than left but these equal controllers worked acceptably
  right_pid.initialise( 15.0, 0.1, 0.0); //kp ki kd
  heading.initialise(1.0, 0.0, 0.0); // was 0.6
  left_pid_rev.initialise( 15.0, 0.1, 0.0); // Right motor has noticible deadband when reversing so needed split controllers
  right_pid_rev.initialise( 18.0, 0.1, 0.0); //kp ki kd
  left_pid.reset();
  right_pid.reset();
  left_pid_rev.reset();
  right_pid_rev.reset();
  heading.reset();

  // Initialise motors, line sensors, encoders, positions
  motors.initialise();
  line_sensors.initialiseForADC();
  setupEncoder0();
  setupEncoder1();

  pose.initialise(0, 0, 0);

  // Making sure magnetometer initialises
  if (!mag.init()) {
    while (1) {
      Serial.println("Magnetometer not detected!");
      delay(1000);
    }
  }
  mag.enableDefault();

  // Set up stopwatch
  display.setMaxMinutes(4);
  display.reset();
  display.startStopwatch();

  // Calibration Routine, allowing time for settling after calibration
  Serial.println("Starting calibration...");
  calibrateSensors();
  Serial.println("Calibration complete.");
  unsigned long settle_start = millis();
  while (millis() - settle_start < 1000) {
    pose.update(); // Ensure that the position is tracked when stationary to try anf give best chance of 'truth'
    delay(10);
  }

  // Start speed estimate, waypoints and patrol status
  speed_est_ts = millis();
  waypoint_index = 1;
  currentState = PATROL;
}



void loop() {
  // Read sensors ASAP
  line_sensors.calcCalibratedADC();

  // Update Pose, speed and M_val collectively
  unsigned long elapsed_time = millis() - speed_est_ts;
  if (elapsed_time > SPEED_EST_MS) {
    speedcalc(elapsed_time);
    speed_est_ts = millis();
    pose.update();
    calcCalibratedM();
  }

  // Stop beeps that might be occuring
  checkBeep();

  // Hard stop if time remaining is 0
  if (!display.timeRemaining()) {
    stoprobot();
    currentState = END;
  }

  // Main FSM Logic
  switch (currentState) {
    case PATROL:
      // Go to waypoint
      GoToXY(x[waypoint_index], y[waypoint_index]);

      // If at waypoint stop and change waypoint
      if (travel_state == TRAVEL_DONE ) {
        stoprobot();

        waypoint_index = waypoint_index + 1;
        if ( waypoint_index >= NUM_WAYPOINTS ) { // reset back to home if cycle complete
          travel_state = TRAVEL_IDLE;
          currentState = GOHOME;
        }
      }

      // If Magnet nearby stop loop and change state
      if (m_val > M_THRESHOLD) {
        setBeep(50); //Found Magnet Beep
        Serial.println("Magnet detected! Beeping and reoreitning...");
        stoprobot();
        travel_state = TRAVEL_IDLE;
        currentState = FOUND;
      }

      break;

    case FOUND: {
        static int found_step = 0;
        static unsigned long move_timer = 0;

        // Static variables to remember the puck's location
        static float p1X, p1Y, p2X, p2Y, p3X, p3Y;
        static float snapX, snapY, snapTheta;

        switch (found_step) {

          case 0: // Snapshot the position where puck was found and Reverse
            if (move_timer == 0) {
              snapX = pose.x;
              snapY = pose.y;
              snapTheta = pose.theta;

              move_timer = millis();
              left_pid_rev.reset();
              right_pid_rev.reset();
            }

            if (millis() - move_timer < 1000) { // 1 second reverse
              float l_pwm = left_pid_rev.update(-0.5, left_speed);
              float r_pwm = right_pid_rev.update(-0.5, right_speed);
              motors.setPWM(l_pwm, r_pwm);
            } else {
              stoprobot();
              move_timer = 0;

              // Manouvre Calculations based on puck locations
              // Where was the puck -> 80mm in front of the snapshot position
              float puckX = snapX + (80.0 * cos(snapTheta));
              float puckY = snapY + (80.0 * sin(snapTheta));

              // Angle from that puck location back to (0,0)
              float angle_to_home = atan2(-puckY, -puckX);

              switch (waypoint_index) { // Skipping parts of the reallignment to minimise turns
                case 1:
                  found_step = 1;
                  break;
                case 2:
                  found_step = 2;
                  break;
                case 3:
                  found_step = 1;
                  break;
                case 4:
                  found_step = 3;
                  break;
                case 5:
                  found_step = 3;
                  break;
                case 6:
                  found_step = 3;
                  break;
                default:
                  found_step = 1;
                  break;
              }

              // Reorienting Geometry
              // P1: Sideways 120mm relative to the puck's radial line to the left
              p1X = puckX + (120.0 * cos(angle_to_home - PI / 2.0));
              p1Y = puckY + (120.0 * sin(angle_to_home - PI / 2.0));

              // P2: Corner Point - Sideways and Behind
              p2X = p1X + (150.0 * cos(angle_to_home + PI));
              p2Y = p1Y + (150.0 * sin(angle_to_home + PI));

              // P3: Directly Behind the puck
              p3X = puckX + (150.0 * cos(angle_to_home + PI));
              p3Y = puckY + (150.0 * sin(angle_to_home + PI));

              travel_state = TRAVEL_IDLE;
            }
            break;

          case 1: // Move to Side Point
            GoToXY(p1X, p1Y);
            if (travel_state == TRAVEL_DONE) {
              found_step = 2;
              travel_state = TRAVEL_IDLE;
            }
            break;

          case 2: // Move to Corner
            GoToXY(p2X, p2Y);
            if (travel_state == TRAVEL_DONE) {
              found_step = 3;
              travel_state = TRAVEL_IDLE;
            }
            break;

          case 3: // Move Behind
            GoToXY(p3X, p3Y);
            if (travel_state == TRAVEL_DONE) {
              stoprobot();
              currentState = PUSHHOME;
              found_step = 0;
              travel_state = TRAVEL_IDLE;
            }
            break;
        }
        break;
      }

    case GOHOME: // Drive Back to 0,0 and set up the pause at 0,0
      GoToXY(0, 0);

      if (travel_state == TRAVEL_DONE) {
        Serial.println("Arrived Home.");
        setBeep(100); // Beep for success - roll dice
        currentState = STOPPED;
        home_ts = millis();
      }
      break;

    case PUSHHOME: // Drive Back to 0,0 and set up the pause at 0,0
      GoToXY(0, 0);

      if (travel_state == TRAVEL_DONE) {
        Serial.println("Arrived Home.");
        setBeep(100); // Beep for success - roll dice
        currentState = STOPPED;
        home_ts = millis();
      }
      break;

    case STOPPED: // Paused at home so wait for 4 seconds to pass then go back on patrol
      stoprobot();
      if (millis() - home_ts > Pause_MS) {
        waypoint_index = 0;
        currentState = PATROL;
      }
      break;

    case END:
      stoprobot();
      break;

    case DEBUG:
      //runSpinTest(); // Checks calibration of wheel seperation by rotating on spot 10 times - best to run without calibration of sensors

      // Checking internal positions whilst testing either x,y or theta
      //  if (millis() - oled_ts > OLED_UPDATE_MS) {
      //    drawPoseOLED();
      //    oled_ts = millis();
      //  }

      // Used to check the M_threshold
      // Serial.print("M_Val: ");
      // Serial.println(m_val);

      // Use the PID to maintain a straight line backwards
      float l_pwm = left_pid_rev.update(-0.5, left_speed);
      float r_pwm = right_pid_rev.update(-0.5, right_speed);
      motors.setPWM(l_pwm, r_pwm);

      break;

  }
}

//void runSpinTest() {
//  static float last_theta = 0;
//  static float accumulated_theta = 0;
//  if (!test_started) {
//    pose.initialise(0, 0, 0); // Why need to run without calibrate sesnors otherwise theta will be difficult to measure
//    accumulated_theta = 0;
//    last_theta = 0;
//    test_started = true;
//  }
//
//  // Calculate change in angle accounting for wrapped theta
//  float diff = angleDiff(pose.theta, last_theta);
//  accumulated_theta += diff;
//  last_theta = pose.theta;
//
//  if (abs(accumulated_theta) < (20.0 * PI)) { // 10 turns
//    float turn_demand = 0.4; // Slow and steady turns
//    float l_pwm = left_pid.update(-turn_demand, left_speed);
//    float r_pwm = right_pid.update(turn_demand, right_speed);
//    motors.setPWM(l_pwm, r_pwm);
//  } else {
//    stoprobot();
//    Serial.println("TEST COMPLETE");
//    Serial.print("Final OLED X: ");
//    Serial.println(pose.x);
//    Serial.print("Final OLED Y: ");
//    Serial.println(pose.y);
//  }
//}
//
//void drawPoseOLED() {
//  display.clear();  // Clear the screen
//
//  display.gotoXY(0, 0);         // Row 1
//  display.print("X: ");
//  display.print(pose.x, 1);
//
//  display.gotoXY(0, 1);         // Row 2
//  display.print("Y: ");
//  display.print(pose.y, 1);
//  /*
//    display.gotoXY(0, 0);       // Row 1
//    display.print("Th: ");
//    display.print(pose.theta * 180.0 / PI, 1); // Convert rad to deg
//  */
//  Serial.print("X: ");
//  Serial.print(pose.x, 4);   // 2 decimal places
//  Serial.print(" mm, Y: ");
//  Serial.print(pose.y, 4);
//  Serial.print(" mm, Theta: ");
//  Serial.print(pose.theta * 180.0 / PI, 4);  // Convert rad → deg for readability
//  Serial.print(" deg, Left: "); // Reading counts of motors to check for unbalanced effective radii
//  Serial.print(abs(count_e1));
//  Serial.print(" , Right: ");
//  Serial.println(abs(count_e0));
//}


// Sensor calibration routine
void calibrateSensors() {
  // Init min / max values for Line and Mag
  for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
    line_sensors.maximum[sensor] = 0;
    line_sensors.minimum[sensor] = 1023;
  }
  for (int i = 0; i < NUM_COORD; i++) {
    M_min[i] = 9999.9;
    M_max[i] = -9999.9;
  }

  // Spinning with ramping speed - giving a set speed demand led to slipping on startup - making drift worse
  unsigned long cal_start_time = millis();
  unsigned long calibration_end = cal_start_time + 4000;
  float max_target_speed = 0.4;
  float current_target = 0;

  while (millis() < calibration_end) {
    unsigned long current_ms = millis();
    unsigned long elapsed = current_ms - speed_est_ts;

    if (elapsed >= SPEED_EST_MS) {
      speedcalc(elapsed);
      speed_est_ts = current_ms;
      pose.update();

      // Speed Ramping from current_target to max_target_speed and down again
      unsigned long time_into_cal = current_ms - cal_start_time;
      if (time_into_cal < 1000) { // Ramp up speed over 1 second
        current_target = ((float)time_into_cal / 1000.0 ) * max_target_speed;
      } else if (calibration_end - current_ms < 500) { // Ramp down at the end
        current_target = ( (float)(calibration_end - current_ms) / 500.0 ) * max_target_speed;
      } else {
        current_target = max_target_speed;
      }

      float l_pwm = left_pid.update(current_target, left_speed);
      float r_pwm = right_pid.update(-current_target, right_speed);
      motors.setPWM(l_pwm, r_pwm);
    }


    // Sensor Calibration logic:
    line_sensors.readSensorsADC();
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      if (line_sensors.readings[sensor] > line_sensors.maximum[sensor])
        line_sensors.maximum[sensor] = line_sensors.readings[sensor];
      if (line_sensors.readings[sensor] < line_sensors.minimum[sensor])
        line_sensors.minimum[sensor] = line_sensors.readings[sensor];
    }
    mag.read();
    int readings[NUM_COORD] = { mag.m.x, mag.m.y, mag.m.z };
    for (int i = 0; i < NUM_COORD; i++) {
      if (readings[i] > M_max[i]) M_max[i] = readings[i];
      if (readings[i] < M_min[i]) M_min[i] = readings[i];
    }
  }

  // Calibration Done - Stop and compute adjustment factors
  stoprobot();
  for ( int sensor = 0; sensor < NUM_SENSORS; sensor++ ) {
    line_sensors.range[sensor] = line_sensors.maximum[sensor] - line_sensors.minimum[sensor];
  }
  for (int i = 0; i < NUM_COORD; i++) {
    M_range[i] = M_max[i] - M_min[i];
    M_offset[i] = (float)M_min[i] + ((float)M_range[i] / 2.0);
    M_scaling[i] = 1 / ((float)M_range[i] / 2.0);
  }
}


// Calculate Calibrated M_value
void calcCalibratedM() {
  mag.read();
  int readings[NUM_COORD] = { mag.m.x, mag.m.y, mag.m.z };
  for (int i = 0; i < NUM_COORD; i++)
    m_calibrated[i] = ((float)readings[i] - M_offset[i]) * M_scaling[i];

  m_val = sqrt(m_calibrated[0] * m_calibrated[0] + m_calibrated[1] * m_calibrated[1] + m_calibrated[2] * m_calibrated[2]);
}

// Returns smallest difference between any two angles
// -ve number indicating a clockwise turn, +ve number is anticlockwise turn
float angleDiff(float target, float source) {
  float diff = target - source;
  if (diff > PI)  diff -= 2.0 * PI;
  if (diff <= -PI) diff += 2.0 * PI;
  return diff;
}

// Stops the motors, resets PID and turns turn and travel flags to false
void stoprobot() {
  motors.setPWM(0, 0);
  Turn_status = false;
  travel_status = false;
  left_pid.reset();  // Clear PID memory
  right_pid.reset();
  heading.reset();
}

// Calculate speeds based on previous pose counts - to have one source of truth
void speedcalc(unsigned long elapsed_time) {
  long delta_e0 = count_e0 - pose.last_e0;
  long delta_e1 = count_e1 - pose.last_e1;

  // Speeds in Counts/ms
  float speed_e0 = float(delta_e0) / float(elapsed_time);
  float speed_e1 = float(delta_e1) / float(elapsed_time);

  // Smoothing
  right_speed = alpha * speed_e0 + (1 - alpha) * last_speed_e0;
  left_speed  = alpha * speed_e1 + (1 - alpha) * last_speed_e1;

  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
}

// Main waypointing logic  - FSM inside a FSM
void GoToXY(float x, float y) {
  // Check if this is a brand new goal or within 4mm if it is, do not change goal
  float dist_to_new_goal = sqrt(pow(x - goal_x, 2) + pow(y - goal_y, 2));
  if (dist_to_new_goal > 4.0) {
    goal_x = x;
    goal_y = y;
    travel_state = TRAVEL_IDLE;
  }

  switch (travel_state) {
    case TRAVEL_IDLE: {
        // Compute desired heading
        float dx = goal_x - pose.x;
        float dy = goal_y - pose.y;
        float theta_D = atan2(dy, dx);
        left_pid.reset();
        right_pid.reset();
        heading.reset();
        // Calculate the shortest path relative to current heading
        float relative_error = angleDiff(theta_D, pose.theta);
        float absolute_target = pose.theta + relative_error;
        // Start the Turn
        setTurn(absolute_target);
        travel_state = TRAVEL_ROTATE;
        break;
      }

    case TRAVEL_ROTATE: {
        if (checkTurn()) {
          // Turn Complete, Start going forward
          setTravel(goal_x, goal_y);
          travel_state = TRAVEL_FORWARD;
          stoprobot();
        }
        break;
      }

    case TRAVEL_FORWARD: {
        if (checkTravel()) {
          // Travel Complete, Set travel done
          travel_state = TRAVEL_DONE;
        }
        break;
      }

    case TRAVEL_DONE: {
        stoprobot();
        break;
      }

    case TRAVEL_ERROR: { // Unused in current build
        motors.setPWM(0, 0);
        break;
      }
  }
}

// Setting up a turn - adjust the global target angle and set flag as true
void setTurn(float new_target_angle) {
  // Prevent retriggering while already turning
  if (Turn_status) return;

  Target_angle = new_target_angle;
  Turn_status = true;
}

// Outputs PWM propotional to angle error
// Return true and stops when angle is within Turn Tolerance
bool checkTurn() {

  float err = angleDiff(Target_angle, pose.theta);
  if ( abs(err) > TURN_TOLERANCE ) {// Have we finished the turn? If not:

    // Speed demand based on error - trying to minimise error
    float demand_turn_speed = heading.update( 0, err );

    // PWM for the motors based on speed diff
    float l_pwm = left_pid.update( demand_turn_speed, left_speed );
    float r_pwm = right_pid.update( -demand_turn_speed, right_speed );

    // Send to Motors
    motors.setPWM( l_pwm, r_pwm );

    // Turn in Progress check turn not done false
    return false;

  } else {
    // Turn finished, stop the robot, reset pid, set flags to false
    stoprobot();
    return true;
  }
}

// Setting up a Travel - adjust the global target x,y and set flag as true
void setTravel(float x, float y) {
  // prevent retriggering if already travelling
  if (travel_status) return;

  goal_x = x;
  goal_y = y;
  travel_status = true;
}


// Outputs PWM propotional to distance error - with minor proprtional heading control to make small adjustments
// Return true and stops when distance is within Tolerance
bool checkTravel() {
  float dx = goal_x - pose.x;
  float dy = goal_y - pose.y;
  float distance = sqrt(dx * dx + dy * dy);

  float current_tolerance = (currentState == PUSHHOME) ? 70.0 : DIST_TOLERANCE;

  if (distance > current_tolerance) {
    float theta_D = atan2(dy, dx);
    float theta_L = angleDiff(theta_D, pose.theta);

    // Scale forward speed based on distance and cap it between 0.2 and 1
    float speed_scale = constrain(distance / 40.0, 0.2, 1.0);
    float base_fwd_demand = 0.5; // Normal Speed - slow but not crawling
    float fwd_demand = base_fwd_demand * speed_scale; // Apply scaling - as robot gets closer, slow down

    // Calculate turn demand
    float turn_demand = 5.0 * theta_L;

    // Update PIDs and sent to motors
    float l_pwm = left_pid.update(fwd_demand - turn_demand, left_speed);
    float r_pwm = right_pid.update(fwd_demand + turn_demand, right_speed);

    motors.setPWM(l_pwm, r_pwm);
    return false;
  } else {
    stoprobot();
    return true;
  }
}

// Set a beep of duration _ms
void setBeep(unsigned long duration_ms) {
  analogWrite(BUZZER_PIN, 120); // 120 is the volume/tone of the beep
  beep_stop_time = millis() + duration_ms;
}

// Checking and stopping beep
void checkBeep() {
  if (millis() > beep_stop_time) {
    analogWrite(BUZZER_PIN, 0); // Turn off
  }
}
