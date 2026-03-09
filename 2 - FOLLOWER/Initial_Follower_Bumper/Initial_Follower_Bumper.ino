// Code Used on ML Robot and ?? Robot
// Just attempting to measure IR using Line sensors on follower

#include "Motors.h"
#include "BumpSensors.h"

Motors_c motors;
BumpSensors_c bump_sensors;

float baseline = 0;

uint32_t lastLoop = 0;
const int LOOP_TIME = 20;


void setup()
{
  Serial.begin(9600);

  motors.initialise();
  bump_sensors.initialiseForADC();


  delay(1000);
  Serial.println("Ready");
  delay(1000);
  lastLoop = millis();
}

void loop()
{
  if(millis() - lastLoop < LOOP_TIME) return;
  lastLoop += LOOP_TIME;

  bump_sensors.readSensorsADC();

  float raw = bump_sensors.readings[0];
  
  //float R = bump_sensors.readings[1];
  //float diff = R - L;


  if(raw > 200){
    motors.setPWM(50,50);  // move forward toward beacon
  }
  else{
    motors.setPWM(0,0);    // no beacon
  }

  
  // Logging
  Serial.println(raw);// Serial.print(",");
  //Serial.print(R); Serial.print(",");
  //Serial.println(diff); 
//  Serial.print(pwmL); Serial.print(",");
//  Serial.println(pwmR);
}
