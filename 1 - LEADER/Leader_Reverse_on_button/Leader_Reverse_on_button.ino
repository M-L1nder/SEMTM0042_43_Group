#include "PID.h"
#include "Motors.h"

#include "Encoders.h"

Motors_c motors;
PID_c left_pid;             // To control the left motor
PID_c right_pid;            // To control the right motor

#define BUTTON_A_PIN 14
#define EMIT_PIN   11


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

unsigned long lastControl; // timestamp for speed estimation - used for pose updates also
#define CONTROL_PERIOD_MS 10     // 10ms

enum LeaderState {
  WAITING,
  REVERSING
};

LeaderState state = WAITING;

float target_left_speed = 0;
float target_right_speed = 0;

bool lastButtonAState = HIGH;


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

  left_pid.initialise( 8.0, 0.1, 0.0); // Left motor does start slightly higher pwm than left but these equal controllers worked acceptably
  right_pid.initialise( 8.0, 0.1, 0.0); //kp ki kd
  left_pid.reset();
  right_pid.reset();

  // Leader beacon ON
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  lastControl = millis();
}

void loop() {
  bool currentButtonAState = digitalRead(BUTTON_A_PIN);

  if (lastButtonAState == HIGH && currentButtonAState == LOW) {
    delay(20); // simple debounce
    if (digitalRead(BUTTON_A_PIN) == LOW) {
      stoprobot();

      if (state == WAITING) {
        state = REVERSING;
      } else {
        state = WAITING;
      }
    }
  }

  lastButtonAState = currentButtonAState;

  if (state == WAITING) {
    stoprobot();
  }
  else if (state == REVERSING) {
    target_left_speed = -0.5;
    target_right_speed = -0.5;
    if (millis() - lastControl >= CONTROL_PERIOD_MS) {
      unsigned long now = millis();
      unsigned long elapsed = now - lastControl;
      lastControl = now;

      speedcalc(elapsed);

      float pwm_left = left_pid.update(target_left_speed, left_speed);
      float pwm_right = right_pid.update(target_right_speed, right_speed);

      motors.setPWM(pwm_left, pwm_right);
    }
  }


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
