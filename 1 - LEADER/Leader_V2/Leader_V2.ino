#include "PID.h"
#include "Motors.h"
#include "Encoders.h"

Motors_c motors;
PID_c left_pid;
PID_c right_pid;

#define BUTTON_A_PIN 14
#define EMIT_PIN 11

// Speed estimation
long last_e0, last_e1;
float last_speed_e0, last_speed_e1;
float right_speed, left_speed;
#define alpha 0.2

unsigned long lastControl = 0;
#define CONTROL_PERIOD_MS 10

enum LeaderState {
  WAITING,
  REVERSING
};

LeaderState state = WAITING;

float target_left_speed = 0.0;
float target_right_speed = 0.0;

bool lastButtonAState = HIGH;
unsigned long reverse_start_ms = 0;
const unsigned long REVERSE_DURATION_MS = 2000;

void setup() {
  Serial.begin(115200);

  motors.initialise();
  setupEncoder0();
  setupEncoder1();

  last_e0 = count_e0;
  last_e1 = count_e1;
  last_speed_e0 = 0;
  last_speed_e1 = 0;

  left_pid.initialise(8.0, 0.1, 0.0);
  right_pid.initialise(8.0, 0.1, 0.0);
  left_pid.reset();
  right_pid.reset();

  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  // Beacon OFF while waiting
  pinMode(EMIT_PIN, INPUT);

  lastControl = millis();
}

void loop() {
  bool currentButtonAState = digitalRead(BUTTON_A_PIN);

  // Toggle start/stop on button press
  if (lastButtonAState == HIGH && currentButtonAState == LOW) {
    delay(20);
    if (digitalRead(BUTTON_A_PIN) == LOW) {
      stoprobot();

      if (state == WAITING) {
        state = REVERSING;
        reverse_start_ms = millis();
      } else {
        state = WAITING;
      }
    }
  }

  lastButtonAState = currentButtonAState;

  if (state == WAITING) {
    // Beacon OFF
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, LOW);
    stoprobot();
    return;
  }

  if (state == REVERSING) {
    // Beacon ON while moving
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, LOW);

    // Auto-stop after 10 seconds
    if (millis() - reverse_start_ms >= REVERSE_DURATION_MS) {
      stoprobot();
      state = WAITING;
      pinMode(EMIT_PIN, INPUT);   // beacon OFF
      return;
    }

    target_left_speed = -0.3;
    target_right_speed = -0.3;

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

void stoprobot() {
  motors.setPWM(0, 0);
  left_pid.reset();
  right_pid.reset();
}

void speedcalc(unsigned long elapsed_time) {
  long delta_e0 = count_e0 - last_e0;
  long delta_e1 = count_e1 - last_e1;

  float speed_e0 = float(delta_e0) / float(elapsed_time);
  float speed_e1 = float(delta_e1) / float(elapsed_time);

  right_speed = alpha * speed_e0 + (1 - alpha) * last_speed_e0;
  left_speed  = alpha * speed_e1 + (1 - alpha) * last_speed_e1;

  last_speed_e0 = right_speed;
  last_speed_e1 = left_speed;
  last_e0 = count_e0;
  last_e1 = count_e1;
}
