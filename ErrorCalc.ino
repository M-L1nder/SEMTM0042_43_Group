// ------------------------------------------------------------------ //
//                       REVERSE 20CM WITH PID                        //
// ------------------------------------------------------------------ //

// ------------------------- Libraries -------------------------------//

#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "PID.h"
#include <Wire.h>

// ------------------------------------------------------------------ //
//                               OBJECTS                              //
// ------------------------------------------------------------------ //
Motors_c motors;
Kinematics_c pose;
PID_c left_pid;
PID_c right_pid;

#define LED_PIN 13
#define BUZZER_PIN 6

// ------------------------------------------------------------------ //
//                           TIMING PARAMETERS                        //
// ------------------------------------------------------------------ //

#define POSE_MS 20 // Kinematics update 
#define SPEED_MS 10 // Whel speed update
#define LOG_MS 50 // Data Log update 

unsigned long pose_ts = 0;
unsigned long speed_ts = 0;
unsigned long log_ts = 0;

// ------------------------------------------------------------------ //
//                           SPEED MEASUREMENT                        //
// ------------------------------------------------------------------ //

#define ALPHA 0.2

long last_e0;
long last_e1;

float speed_e0 = 0.0;
float speed_e1 = 0.0;
float speed_e0_filt = 0.0;
float speed_e1_filt = 0.0;

// ------------------------------------------------------------------ //
//                           REVERSE CONTROL                          //
// ------------------------------------------------------------------ //

#define REVERSE_DIST 200.0 
#define REVERSE_DEMAND 0.5
float start_x = 0.0;
float start_y = 0.0;
bool reverse_active = false;
bool reverse_done = false;

// ------------------------------------------------------------------ //
//                           DATA LOGGING                             //
// ------------------------------------------------------------------ //

// LOG_MS  = 50 ms for 5 sconds of travel coveers 100 samples

#define MAX_SAMPLES 100

unsigned long log_time[MAX_SAMPLES]; 
float log_x[MAX_SAMPLES];
float log_y[MAX_SAMPLES];

int sample_count = 0;
bool data_printed = false;
unsigned long run_start_ms = 0; // record when rvrsing begins

// ------------------------------------------------------------------ //
//                           WHEEL SPED UPDATE                         //
// ------------------------------------------------------------------ //

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


// ------------------------------------------------------------------ //
//                           FUNCTION: REVERS_PID                     //
// ------------------------------------------------------------------ //

void reversePID(float rev_demand){

  float left_pwm = left_pid.update(-rev_demand, speed_e1_filt);
  float right_pwm = right_pid.update(-rev_demand, speed_e0_filt);

  motors.setPWM((int)left_pwm, (int)right_pwm);
}

// ------------------------------------------------------------------ //
//                           FUNCTION: SAMPLE RECORDING               //
// ------------------------------------------------------------------ //

// STORE TIME, X & Y POSITIONS INTO ARRAYS

void recordSample(){

  unsigned long now = millis();

  if (now - log_ts < LOG_MS){
    return;
  }

  log_ts = now;

  if (sample_count < MAX_SAMPLES){
    log_time[sample_count] = now - run_start_ms;
    log_x[sample_count] = pose.x;
    log_y[sample_count] = pose.y;
    sample_count++;
  }
}


// ------------------------------------------------------------------ //
//                           FUNCTION: PRINT DATA                     //
// ------------------------------------------------------------------ //

void printCSV(){

  if(data_printed){
    return;
  }

  if(!Serial){
    return;
  }

  delay(500);

  //Print CSV Heeader

  Serial.println("time_ms,x_mm,y_mm");

  for(int i =0; i< sample_count; i++){
    Serial.print(log_time[i]);
    Serial.print(",");
    Serial.print(log_x[i],2);
    Serial.print("'");
    Serial.print(log_y[i]),2;
  }

  Serial.println();
  Serial.print("Tota samples: ");
  Serial.print(sample_count);
  Serial.println(" COPY INTO EXCEL FILE ");

  data_printed = true;
}

// ------------------------------------------------------------------ //
//                           SETUP FUNCTION                           //
// ------------------------------------------------------------------ //

void setup(){

  // Serial for debugging.
  // ------------------------

  Serial.begin(9600);
  
  Wire.begin();
  // Initialising Motor and encoder hardware ------
  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  
  // Pose Initialisation
  pose.initialise(0, 0, 0);
 
 
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
  log_ts = millis();

  // Initialise all gains --------------------------
  left_pid.initialise( 60, 0.02, 0.001 );
  right_pid.initialise( 60, 0.02, 0.001 );

  // Remember to reset PID -------------------------
  // used any delay()

  left_pid.reset();
  right_pid.reset();

  sample_count = 0;
  data_printed = false;

  // Delay to place robot after pressing power 
  delay (2000);
  digitalWrite(LED_PIN, HIGH); // led being on indicates robot about to move

  // BEGIN REVERSNG
  start_x = pose.x;
  start_y = pose.y;
  run_start_ms = millis();
  reverse_active = true;
  reverse_done = false;

}

// ------------------------------------------------------------------ //
//                           LOPP  FUNCTION                           //
// ------------------------------------------------------------------ //

void loop(){

  // KINEMATICS UPDATE
  unsigned long now = millis();
  if(now - pose_ts >=POSE_MS){
    pose_ts += POSE_MS;
    pose.update();
  }

  // WHEEL SPEED UPDATE
  unsigned long elapsed = millis - speed_ts;
  if (elapsed >= SPEED_MS){
    updateWheelSpeeds(elapsed);
  }

  // ======================================================//
  // PHASE 1                                               //
  // ======================================================//
  if (reverse_active && !reverse_done){
    recordSample(); // record positons into array

    float dx = pose.x - start_x;
    float dy = pose.y - start_y;
    float dist = sqrt(dx*dx + dy*dy);

     if (dist >=REVERSE_DIST){
      motors.setPWM(0,0);
      reverse_active = false;
      reverse_done = true;

      if(sample_count < MAX_SAMPLES){
        log_time[sample_count] = millis() - run_start_ms;
        log_x[sample_count] = pose.x;
        log_y[sample_count] = pose.y;
        sample_count++;
      }
      left_pid.reset();
      right_pid.reset();

      digitalWrite(LED_PIN,LOW);
      return;

     }
   reversePID(REVERSE_DEMAND);
   return;
  }

  // ======================================================//
  // PHASE 2 -PRINT CSV IN SRIAL MONITOR                   //
  // ======================================================//
  if (reverse_done){
    printCSV();
  }
}
