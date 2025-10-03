  /* Man-following kart + RFID billing system
   Hardware:
    - 2x HC-SR04 (left, right)
    - 2x DC motors with motor driver (L298N/TB6612)
    - MFRC522 RFID
    - I2C 16x2 LCD
    - Arduino Uno

   Behavior:
    - Follow person using left/right ultrasonic sensors (differential steering)
    - Tap RFID to start ride, tap again to stop and charge by time
    - Balances persisted in EEPROM
*/

#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ---------------- PIN CONFIG ----------------
/* Motors (example pins) */
const int ENA = 5;   // PWM left motor
const int IN1 = 8;
const int IN2 = 9;

const int ENB = 6;   // PWM right motor
const int IN3 = 10;
const int IN4 = 11;

/* Ultrasonic sensors */
const int TRIG_L = 2;
const int ECHO_L = 3;
const int TRIG_R = 4;
const int ECHO_R = 7;

/* RFID (MFRC522) */
const int RST_PIN = 9;    // change if conflicts; must not conflict with motor pins
const int SS_PIN  = 10;   // SPI SS (using D10)

// I2C LCD address (commonly 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID object
MFRC522 mfrc522(SS_PIN, RST_PIN);

// ---------------- PARAMETERS ----------------
const long FOLLOW_DISTANCE_CM = 60; // desired distance to person (cm)
const long DIST_TOLERANCE = 12;     // hysteresis
const int MAX_SPEED = 200;          // 0..255 PWM
const int MIN_SPEED = 90;           // base speed to overcome friction
const float TURN_K = 1.7;           // proportional constant for steering

// Billing
const float RATE_PER_MIN = 0.50;    // e.g., $0.50 per minute
const float DEFAULT_BALANCE = 5.00; // default top-up when registering new card

// EEPROM mapping
// Simple schema: up to N users, each record fixed size
const int MAX_USERS = 20;
const int RECORD_SIZE = 12; // e.g. 4 bytes UID hash + 8 bytes float balance (we'll store as int cents)
const int EEPROM_BASE = 0;

// Runtime variables
unsigned long rideStartTime = 0;
bool rideActive = false;
byte activeUID[10];
int activeUIDlen = 0;

struct UserRecord {
  unsigned long uidHash;
  long balanceCents; // store cents to avoid float in EEPROM
};

// ---------------- HELPER FUNCTIONS ----------------

// Motor helpers
void setupMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();
}

void setLeftMotor(int speed) { // speed -255..255
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(speed,0,255));
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-speed,0,255));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setRightMotor(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(speed,0,255));
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-speed,0,255));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

// Ultrasonic helper
long readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout ~30ms (approx 5m)
  if (duration == 0) return 3000; // very far (no echo)
  long cm = duration / 58; // speed of sound
  return cm;
}

// Simple uid hash (not cryptographic) to identify card
unsigned long uidToHash(byte *uid, byte uidLen) {
  unsigned long h = 1469598103934665603UL; // FNV offset (but truncated)
  for (byte i = 0; i < uidLen; i++) {
    h ^= uid[i];
    h *= 1099511628211UL;
  }
  return h;
}

// EEPROM helpers: store up to MAX_USERS records.
// address calculation
int recordAddress(int idx) {
  return EEPROM_BASE + idx * RECORD_SIZE;
}

void writeLongToEEPROM(int addr, unsigned long value) {
  for (int i=0;i<4;i++) {
    EEPROM.update(addr + i, (value >> (8*i)) & 0xFF);
  }
}
unsigned long readLongFromEEPROM(int addr) {
  unsigned long val = 0;
  for (int i=0;i<4;i++) {
    val |= ((unsigned long)EEPROM.read(addr + i)) << (8*i);
  }
  return val;
}

void writeLongLongToEEPROM(int addr, long value) {
  // store 4 bytes for cents (fits within +/-2 billion cents ~ $20 million)
  writeLongToEEPROM(addr, (unsigned long)value);
}

long readLongLongFromEEPROM(int addr) {
  return (long)readLongFromEEPROM(addr);
}

// find user by uidHash
int findUserIndex(unsigned long uidHash) {
  for (int i=0;i<MAX_USERS;i++) {
    int addr = recordAddress(i);
    unsigned long h = readLongFromEEPROM(addr);
    if (h == uidHash) return i;
  }
  return -1;
}

// create new user record at first free slot, return index or -1
int createUser(unsigned long uidHash, long initCents) {
  for (int i=0;i<MAX_USERS;i++) {
    int addr = recordAddress(i);
    unsigned long h = readLongFromEEPROM(addr);
    if (h == 0xFFFFFFFFUL || h == 0UL) { // assume empty (0 or all 0xFF)
      // write hash
      writeLongToEEPROM(addr, uidHash);
      // write balance (4 bytes next)
      writeLongLongToEEPROM(addr + 4, initCents);
      return i;
    }
  }
  return -1;
}

long getBalanceCents(int idx) {
  int addr = recordAddress(idx) + 4;
  return readLongLongFromEEPROM(addr);
}

void setBalanceCents(int idx, long cents) {
  int addr = recordAddress(idx) + 4;
  writeLongLongToEEPROM(addr, cents);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  // pins
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  setupMotors();
  lcd.init();
  lcd.backlight();

  // RFID init
  SPI.begin();
  mfrc522.PCD_Init();
  lcd.clear();
  lcd.print("Kart initializing");
  delay(800);
  lcd.clear();
  lcd.print("Ready - Tap card");
  Serial.println("Ready. Tap a card.");

  // If EEPROM blank, you may optionally zero it or leave as is.
  // We'll treat 0 or 0xFFFFFFFF as empty. (Default EEPROM content depends on board)
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // read ultrasonics
  long dl = readUltrasonicCM(TRIG_L, ECHO_L);
  long dr = readUltrasonicCM(TRIG_R, ECHO_R);
  // debug
  //Serial.print("L:"); Serial.print(dl); Serial.print(" R:"); Serial.println(dr);

  // Follow control logic
  followControl(dl, dr);

  // Check RFID
  handleRFID();

  delay(80); // loop delay, tune as needed
}

// ---------------- FOLLOW CONTROL ----------------
void followControl(long dl, long dr) {
  // If ride isn't active, we should not move. Kart only moves during active ride.
  if (!rideActive) {
    stopMotors();
    return;
  }

  // average distance
  long avg = (dl + dr) / 2;
  long error = dl - dr; // >0 means person more to right (closer on left), so steer right

  // Decide forward/back
  int baseSpeed = 0;
  if (avg > FOLLOW_DISTANCE_CM + DIST_TOLERANCE) {
    baseSpeed = MIN_SPEED; // move forward at base speed
  } else if (avg < FOLLOW_DISTANCE_CM - DIST_TOLERANCE) {
    // too close: stop or back off
    baseSpeed = 0;
  } else {
    baseSpeed = 0; // hold
  }

  // Steering proportional
  int steer = (int)(TURN_K * error); // could be negative
  // Left motor = baseSpeed - steer; Right motor = baseSpeed + steer
  int leftSpeed = baseSpeed - steer;
  int rightSpeed = baseSpeed + steer;

  // Constrain to PWM limits
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // If baseSpeed is 0 but steering non-zero, you might want to pivot slightly. Here we keep stationary.
  if (baseSpeed == 0) {
    stopMotors();
  } else {
    setLeftMotor(leftSpeed);
    setRightMotor(rightSpeed);
  }
}

// ---------------- RFID / Billing ----------------
void handleRFID() {
  // Look for new card
  if ( ! mfrc522.PICC_IsNewCardPresent()) return;
  if ( ! mfrc522.PICC_ReadCardSerial()) return;

  // read UID
  byte uidLen = mfrc522.uid.size;
  byte uid[10];
  for (byte i=0;i<uidLen;i++) uid[i] = mfrc522.uid.uidByte[i];

  unsigned long h = uidToHash(uid, uidLen);
  int idx = findUserIndex(h);

  if (idx < 0) {
    // register new user with default balance
    long cents = (long)(DEFAULT_BALANCE * 100.0 + 0.5);
    idx = createUser(h, cents);
    if (idx >= 0) {
      lcd.clear();
      lcd.print("New card added");
      lcd.setCursor(0,1);
      lcd.print("Bal $"); lcd.print(DEFAULT_BALANCE, 2);
      delay(1300);
    } else {
      lcd.clear(); lcd.print("Max users full");
      delay(1000);
      return;
    }
  }

  long balCents = getBalanceCents(idx);
  float bal = balCents / 100.0;

  // if no active ride: start
  if (!rideActive) {
    // require minimum balance
    float minReq = 0.50; // require at least 50 cents to start
    if (bal < minReq) {
      lcd.clear();
      lcd.print("Insuff bal $");
      lcd.print(bal,2);
      delay(1200);
      return;
    }
    // start ride
    rideActive = true;
    rideStartTime = millis();
    memcpy(activeUID, uid, uidLen);
    activeUIDlen = uidLen;
    lcd.clear();
    lcd.print("Ride started");
    lcd.setCursor(0,1);
    lcd.print("Bal $"); lcd.print(bal,2);
    Serial.println("Ride started");
    delay(800);
  } else {
    // if ride active, check if same card to stop
    unsigned long activeHash = uidToHash(activeUID, activeUIDlen);
    if (activeHash == h) {
      // stop ride
      unsigned long t = millis();
      unsigned long dt = t - rideStartTime; // ms
      float minutes = dt / 60000.0;
      long chargeCents = (long)(minutes * RATE_PER_MIN * 100.0 + 0.5);
      if (chargeCents < 1) chargeCents = 1; // minimum 1 cent
      if (chargeCents > balCents) chargeCents = balCents; // don't go negative (optional)
      long newBal = balCents - chargeCents;
      setBalanceCents(idx, newBal);
      float newBalF = newBal / 100.0;

      // display
      lcd.clear();
      lcd.print("Ride ended");
      lcd.setCursor(0,1);
      lcd.print("Chg $"); lcd.print(char(chargeCents/100)); // we'll display nicely below

      // better display formatting
      lcd.clear();
      lcd.print("Duration:");
      lcd.print(minutes, 2);
      lcd.setCursor(0,1);
      lcd.print("Charged:$");
      lcd.print(chargeCents/100.0,2);
      delay(1400);

      lcd.clear();
      lcd.print("New bal $");
      lcd.print(newBalF,2);
      Serial.print("Ride ended. Charged $"); Serial.print(chargeCents/100.0,2);
      Serial.print(" New bal $"); Serial.println(newBalF,2);
      delay(1300);

      // reset ride state
      rideActive = false;
      activeUIDlen = 0;
      memset(activeUID,0,sizeof(activeUID));
      stopMotors();
    } else {
      // Another card tapped during an active ride
      lcd.clear();
      lcd.print("Ride active");
      lcd.setCursor(0,1);
      lcd.print("Tap same card to stop");
      delay(1200);
    }
  }

  // done with card
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

// ---------------- END OF CODE ----------------
