// Code Used on ML Robot and ?? Robot
// Just attempting to measure IR using Line sensors on follower

#include "Motors.h"
#include "LineSensors.h"

Motors_c motors;
LineSensors_c line_sensors;

float baseline = 0;

uint32_t lastLoop = 0;
const int LOOP_TIME = 20;


void setup()
{
  Serial.begin(115200);

  motors.initialise();
  line_sensors.initialiseForADC();

  // Turn OFF this robot's emitters
  pinMode(EMIT_PIN, INPUT);
  calibrateIRReceivers();

  delay(10000);
  Serial.println("Ready");
  delay(5000);
  lastLoop = millis();
}

void loop()
{
  if(millis() - lastLoop < LOOP_TIME) return;
  lastLoop += LOOP_TIME;

  line_sensors.calcCalibratedADC();

  float L = line_sensors.readings[0] + line_sensors.readings[1];
  float C = line_sensors.readings[2];
  float R = line_sensors.readings[3] + line_sensors.readings[4];

  float total = L + C + R;
  
  float diff = R - L;


  // Logging
  Serial.print(L); Serial.print(",");
  Serial.print(C); Serial.print(",");
  Serial.print(R); Serial.print(",");
  Serial.println(total); 
//  Serial.print(pwmL); Serial.print(",");
//  Serial.println(pwmR);
}


const int CAL_TIME_MS = 2500;     // 2.5s spin calibration
const int CAL_PWM     = 60;       // spin speed


// Storage for normalized values
float norm[NUM_SENSORS];

void calibrateIRReceivers()
{
  // Ensure follower isn't emitting its own IR
  pinMode(EMIT_PIN, INPUT);

  // Init min/max
  for (int s = 0; s < NUM_SENSORS; s++) {
    line_sensors.maximum[s] = 0;
    line_sensors.minimum[s] = 1023;
  }

  uint32_t t0 = millis();

  // Spin in place while calibrating
  motors.setPWM(-CAL_PWM, CAL_PWM);

  while (millis() - t0 < CAL_TIME_MS)
  {
    line_sensors.readSensorsADC();

    for (int s = 0; s < NUM_SENSORS; s++) {
      int v = line_sensors.readings[s];
      if (v > line_sensors.maximum[s]) line_sensors.maximum[s] = v;
      if (v < line_sensors.minimum[s]) line_sensors.minimum[s] = v;
    }

    delay(10);
  }

  motors.setPWM(0, 0);

  // Compute range with a floor to avoid divide-by-zero / tiny ranges
  for (int s = 0; s < NUM_SENSORS; s++) {
    int r = line_sensors.maximum[s] - line_sensors.minimum[s];
    if (r < 20) r = 20;             // floor range (tune if needed)
    line_sensors.range[s] = r;
  }

  // Optional: print calibration results
  Serial.println("CAL DONE: min,max,range");
  for (int s = 0; s < NUM_SENSORS; s++) {
    Serial.print(s); Serial.print(":");
    Serial.print(line_sensors.minimum[s]); Serial.print(",");
    Serial.print(line_sensors.maximum[s]); Serial.print(",");
    Serial.println(line_sensors.range[s]);
  }
}
