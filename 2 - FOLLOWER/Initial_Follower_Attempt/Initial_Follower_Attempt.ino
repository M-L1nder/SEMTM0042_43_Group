// Code Used on ML Robot and ?? Robot
// Just attempting to measure IR using Line sensors on follower

#include "Motors.h"
#include "LineSensors.h"

Motors_c motors;
LineSensors_c line_sensors;

float baseline = 0;

//float Kturn = 0.05;
//float Kfwd  = 0.02;

uint32_t lastLoop = 0;
const int LOOP_TIME = 20;

float measureBaseline()
{
  float total = 0;

  for(int i=0;i<50;i++)
  {
    line_sensors.readSensorsADC();

    float L = line_sensors.readings[0] + line_sensors.readings[1];
    float C = line_sensors.readings[2];
    float R = line_sensors.readings[3] + line_sensors.readings[4];

    total += (L+C+R);

    delay(5);
  }

  return total/50.0;
}

void setup()
{
  Serial.begin(115200);

  motors.initialise();
  line_sensors.initialiseForADC();

  // Turn OFF this robot's emitters
  pinMode(EMIT_PIN, INPUT);

  baseline = measureBaseline();

  Serial.print("Baseline = ");
  Serial.println(baseline);

  lastLoop = millis();
}

void loop()
{
  if(millis() - lastLoop < LOOP_TIME) return;
  lastLoop += LOOP_TIME;

  line_sensors.readSensorsADC();

  float L = line_sensors.readings[0] + line_sensors.readings[1];
  float C = line_sensors.readings[2];
  float R = line_sensors.readings[3] + line_sensors.readings[4];

  float total = L + C + R;
  float signal = total - baseline;

  float diff = R - L;

//  float forward = 0;
//  float turn = 0;
//
//  if(signal < 50)
//  {
//    // search behaviour
//    forward = 0;
//    turn = 30;
//  }
//  else
//  {
//    turn = Kturn * diff;
//    forward = Kfwd * signal;
//
//    if(forward > 80) forward = 80;
//    if(forward < 0) forward = 0;
//  }
//
//  float pwmL = forward - turn;
//  float pwmR = forward + turn;
//
//  motors.setPWM(pwmL, pwmR);

  // Logging
  Serial.print(millis());
  Serial.print(",");
  Serial.print(L); Serial.print(",");
  Serial.print(C); Serial.print(",");
  Serial.print(R); Serial.print(",");
  Serial.println(signal); 
//  Serial.print(pwmL); Serial.print(",");
//  Serial.println(pwmR);
}
