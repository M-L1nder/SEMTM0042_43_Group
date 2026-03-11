#include "Motors.h"
#include "LineSensors.h"

Motors_c motors;

// button pin
const int BUTTON_A_PIN = 14;

// motion
const int REVERSE_SPEED = 20;
const uint32_t REVERSE_TIME_MS = 5000;

// state
bool reversing = false;
bool lastButtonState = HIGH;
uint32_t reverseStartTime = 0;

void setup()
{
  motors.initialise();

  // turn IR emitters on immediately
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);

  // button input with pull-up
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);

  // start still
  motors.setPWM(0, 0);
}

void loop()
{
  bool buttonState = digitalRead(BUTTON_A_PIN);

  // detect a new button press: HIGH -> LOW
  if (!reversing && lastButtonState == HIGH && buttonState == LOW) {
    reversing = true;
    reverseStartTime = millis();
    motors.setPWM(-REVERSE_SPEED, -REVERSE_SPEED);
  }

  // stop after 5 seconds
  if (reversing && (millis() - reverseStartTime >= REVERSE_TIME_MS)) {
    reversing = false;
    motors.setPWM(0, 0);
  }

  lastButtonState = buttonState;
}
