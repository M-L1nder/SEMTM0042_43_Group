//#define EMIT_PIN 11
//#define BUMP_RX_PIN 5   // your ADC-capable bump receiver pin
//
//// Choose a timeout. Start with 3000–6000 us.
//// Far away / weak IR => longer discharge time.
//// Very strong IR => short discharge time.
//const unsigned long TIMEOUT_US = 6000;
//
//void setup() {
//  pinMode(EMIT_PIN, INPUT);       // follower: emitters OFF
//  pinMode(BUMP_RX_PIN, INPUT);    // start as input
//
//  Serial.begin(115200);
//  delay(1500);
//  Serial.println("***RESET***");
//}
//
//unsigned long readDischargeTimeOnce()
//{
//  // (Optional) If you want follower to illuminate with its own bump LEDs, use:
//  // pinMode(EMIT_PIN, OUTPUT); digitalWrite(EMIT_PIN, LOW);
//  // For leader-following, usually keep emitters OFF:
//  pinMode(EMIT_PIN, INPUT);
//
//  // Charge the sensor node
//  pinMode(BUMP_RX_PIN, OUTPUT);
//  digitalWrite(BUMP_RX_PIN, HIGH);
//  delayMicroseconds(10);
//
//  // Switch to input so it can discharge
//  pinMode(BUMP_RX_PIN, INPUT);
//
//  unsigned long start_time = micros();
//
//  // Wait until it discharges (goes LOW), but with timeout
//  while (digitalRead(BUMP_RX_PIN) == HIGH) {
//    if (micros() - start_time > TIMEOUT_US) {
//      return TIMEOUT_US; // timed out: treat as "very weak signal"
//    }
//  }
//
//  unsigned long end_time = micros();
//  return end_time - start_time;
//}
//
//void loop() {
//  unsigned long t = readDischargeTimeOnce();
//  Serial.println(t);
//  delay(20);
//}



// DOUBLE TEST
#define EMIT_PIN 11

#define BUMP_PIN_A6 A6
#define BUMP_PIN_5  5

const unsigned long TIMEOUT_US = 6000;

unsigned long readDischargeTime(uint8_t pin)
{
  // Follower: emitters OFF so we see leader beacon + ambient only
  pinMode(EMIT_PIN, INPUT);

  // Charge
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);

  // Discharge
  pinMode(pin, INPUT);

  unsigned long start = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - start > TIMEOUT_US) return TIMEOUT_US;
  }

  return micros() - start;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("tA6,t5,diff");
}

void loop() {
  unsigned long tA6 = readDischargeTime(BUMP_PIN_A6);
  unsigned long t5  = readDischargeTime(BUMP_PIN_5);

  long diff = (long)t5 - (long)tA6;   // sign is arbitrary; you can flip if needed

  Serial.print(tA6);
  Serial.print(",");
  Serial.print(t5);
  Serial.print(",");
  Serial.println(diff);

  delay(20);
}
