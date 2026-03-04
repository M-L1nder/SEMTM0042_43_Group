// Code Used on ML Robot and ?? Robot
// Just attempting to output IR forwards to be detectable by Follower
// IR to start after button A pressed

#include "Motors.h"
#include "LineSensors.h"

Motors_c motors;

void setup() {

  motors.initialise();

  // Turn ON bump IR emitters
  pinMode(EMIT_PIN, OUTPUT);
  digitalWrite(EMIT_PIN, LOW);  

}

void loop() {

  // Keep robot stationary (best for first tests)
  motors.setPWM(0,0);

  // Later you can try moving slowly
  // motors.setPWM(40,40);

}
