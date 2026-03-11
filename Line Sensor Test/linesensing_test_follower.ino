#include "Motors.h"
#include "LineSensors.h"

Motors_c motors;
LineSensors_c line_sensors;

// timing
uint32_t lastLoop = 0;
const int LOOP_TIME_MS = 20;

// calibration
const int CAL_TIME_MS = 3000;
const int CAL_PWM = 35;

// motion
const float FORWARD_SPEED = 50.0;
const float ALIGN_TURN_SPEED = 28.0;

// alignment tuning
const float LR_TOLERANCE = 5.0;          // how close L and R must be
const int ALIGN_STABLE_MS = 500;         // must stay aligned for this long

// threshold tuning
const float BAND_MARGIN_FRACTION = 0.15;
const float MIN_BAND_MARGIN = 15.0;

// grouped values
float total = 0;
float L = 0;
float R = 0;
float C = 0;
float lrError = 0;

// control thresholds
float targetTotal = 0;
float lowerThreshold = 0;
float upperThreshold = 0;

// state
enum RobotState {
  ALIGNING,
  FOLLOWING
};

RobotState state = ALIGNING;
uint32_t alignedSince = 0;

// helper
float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void updateSensors()
{
  line_sensors.readSensorsADC();

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = line_sensors.readings[i];
    int minv = line_sensors.minimum[i];
    int range = line_sensors.range[i];

    if (range < 1) range = 1;

    float norm = (float)(raw - minv) / (float)range;
    norm = clampFloat(norm, 0.0, 1.0);

    line_sensors.readings[i] = norm * 100.0;
  }

  L = line_sensors.readings[0] + line_sensors.readings[1];
  C = line_sensors.readings[2];
  R = line_sensors.readings[3] + line_sensors.readings[4];

  total = L + C + R;
  lrError = R - L;
}

void calibrateDistanceBand()
{
  pinMode(EMIT_PIN, INPUT);

  for (int i = 0; i < NUM_SENSORS; i++) {
    line_sensors.minimum[i] = 1023;
    line_sensors.maximum[i] = 0;
    line_sensors.range[i] = 1;
  }

  uint32_t startTime = millis();

  // rotate during calibration to capture useful min/max values
  motors.setPWM(-CAL_PWM, CAL_PWM);

  while (millis() - startTime < CAL_TIME_MS) {
    line_sensors.readSensorsADC();

    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = line_sensors.readings[i];

      if (v < line_sensors.minimum[i]) line_sensors.minimum[i] = v;
      if (v > line_sensors.maximum[i]) line_sensors.maximum[i] = v;
    }

    delay(10);
  }

  motors.setPWM(0, 0);

  for (int i = 0; i < NUM_SENSORS; i++) {
    int r = line_sensors.maximum[i] - line_sensors.minimum[i];
    if (r < 20) r = 20;
    line_sensors.range[i] = r;
  }

  // sample total at desired distance
  float minTotalSeen = 100000.0;
  float maxTotalSeen = -100000.0;
  float sumTotal = 0.0;
  int count = 0;

  startTime = millis();
  while (millis() - startTime < CAL_TIME_MS) {
    updateSensors();

    if (total < minTotalSeen) minTotalSeen = total;
    if (total > maxTotalSeen) maxTotalSeen = total;

    sumTotal += total;
    count++;

    delay(10);
  }

  if (count < 1) count = 1;

  targetTotal = sumTotal / (float)count;

  float observedSpread = maxTotalSeen - minTotalSeen;
  float bandMargin = observedSpread * BAND_MARGIN_FRACTION;
  if (bandMargin < MIN_BAND_MARGIN) bandMargin = MIN_BAND_MARGIN;

  lowerThreshold = targetTotal - bandMargin;
  upperThreshold = targetTotal + bandMargin;

  Serial.println("calibration done");
  Serial.print("targetTotal,"); Serial.println(targetTotal);
  Serial.print("lowerThreshold,"); Serial.println(lowerThreshold);
  Serial.print("upperThreshold,"); Serial.println(upperThreshold);
}

void setup()
{
  Serial.begin(115200);

  motors.initialise();
  line_sensors.initialiseForADC();

  pinMode(EMIT_PIN, INPUT);

  delay(500);
  calibrateDistanceBand();

  state = ALIGNING;
  alignedSince = 0;

  delay(500);
  lastLoop = millis();
}

void loop()
{
  if (millis() - lastLoop < LOOP_TIME_MS) return;
  lastLoop += LOOP_TIME_MS;

  updateSensors();

  float leftPWM = 0;
  float rightPWM = 0;

  if (state == ALIGNING) {
    // rotate until L and R are close enough
    if (fabs(lrError) <= LR_TOLERANCE) {
      if (alignedSince == 0) {
        alignedSince = millis();
      }

      if (millis() - alignedSince >= ALIGN_STABLE_MS) {
        state = FOLLOWING;
      }

      leftPWM = 0;
      rightPWM = 0;
    } else {
      alignedSince = 0;

      if (lrError > 0) {
        // right stronger -> turn right
        leftPWM = -ALIGN_TURN_SPEED;
        rightPWM = ALIGN_TURN_SPEED;
      } else {
        // left stronger -> turn left
        leftPWM = ALIGN_TURN_SPEED;
        rightPWM = -ALIGN_TURN_SPEED;
      }
    }
  }
  else if (state == FOLLOWING) {
    // simple straight-line follow logic
    if (C > 30) {
      leftPWM = FORWARD_SPEED;
      rightPWM = FORWARD_SPEED;
    } else {
      leftPWM = 0;
      rightPWM = 0;
    }
  }

  leftPWM = clampFloat(leftPWM, -100, 100);
  rightPWM = clampFloat(rightPWM, -100, 100);

  motors.setPWM((int)leftPWM, (int)rightPWM);

  Serial.print(total);
  Serial.print(",");
  Serial.print(L);
  Serial.print(",");
  Serial.print(C);
  Serial.print(",");
  Serial.print(R);
  Serial.print(",");
  Serial.print(lrError);
  Serial.print(",");
  Serial.println(state == ALIGNING ? 1 : 2);
}
