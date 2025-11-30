# smart-blind-stick-for-visually-impaired- 
/* Smart Blind Stick - Arduino UNO Full Code
   Components supported:
   - HC-SR04 Ultrasonic (obstacle detection)
   - IR sensor (close-range)
   - Water sensor (digital)
   - LDR (darkness detection) -> LED torch auto ON
   - GPS NEO-6M (location)
   - GSM SIM800L (send SMS in emergency)
   - Buzzer (alerts)
   - Vibration motor (alerts)
   - ON/OFF switch (power on/off + long press emergency)
   - Uses SoftwareSerial for GPS & GSM
   Libraries required:
     - TinyGPSPlus (Install from Library Manager)
*/

#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// ---------------- PIN DEFINITIONS ----------------
const int TRIG_PIN   = 11;
const int ECHO_PIN   = 10;

const int GPS_RX_PIN = 4;   // Connect GPS TX -> Arduino 4
const int GPS_TX_PIN = 5;   // (optional) GPS RX -> Arduino 5 (not always needed)

const int GSM_RX_PIN = 7;   // Connect GSM TX -> Arduino 7
const int GSM_TX_PIN = 8;   // Connect GSM RX -> Arduino 8

const int IR_PIN     = 2;
const int WATER_PIN  = 3;
const int BUZZER_PIN = 9;
const int VIB_PIN    = A1;  // use analog pin as digital
const int FLASH_PIN  = A2;  // torch LED (use transistor if LED high current)
const int LDR_PIN    = A0;  // analog input for light sensor
const int SWITCH_PIN = A3;  // power / emergency button

// ---------------- SENSOR THRESHOLDS & TIMINGS ----------------
const unsigned long ULTRASONIC_INTERVAL = 200; // ms between readings
const long DISTANCE_ALERT_CM = 120;   // if object within this distance -> alert
const int LDR_THRESHOLD = 400;        // analog threshold (0-1023). lower = darker
const unsigned long EMERGENCY_HOLD_MS = 2000; // hold ON/OFF for this long to trigger emergency

// ---------------- COMMUNICATION & OBJECTS ----------------
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX
SoftwareSerial gsmSerial(GSM_RX_PIN, GSM_TX_PIN); // RX, TX
TinyGPSPlus gps;

// Phone no for emergency SMS (change to your trusted contact)
String EMERGENCY_NUMBER = "+911234567890"; // <-- Replace with real number including country code

// ---------------- STATE VARIABLES ----------------
unsigned long lastUltrasonic = 0;
bool systemOn = false;
unsigned long switchPressedTime = 0;
bool switchWasPressed = false;

// ---------------- HELPER FUNCTIONS ----------------
void beep(int times, int dur = 120) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(dur);
    digitalWrite(BUZZER_PIN, LOW);
    delay(80);
  }
}

void vibrate(int ms) {
  digitalWrite(VIB_PIN, HIGH);
  delay(ms);
  digitalWrite(VIB_PIN, LOW);
}

long readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  // duration/2 /29.1 = cm
  long distanceCm = duration / 29 / 2;
  if (duration == 0) return -1; // no echo
  return distanceCm;
}

// Send AT command and optionally wait for a particular response (simple)
bool sendATexpect(const char *cmd, const char *expect, unsigned long timeout = 2000) {
  while (gsmSerial.available()) gsmSerial.read(); // flush
  gsmSerial.println(cmd);
  unsigned long start = millis();
  String res = "";
  while (millis() - start < timeout) {
    while (gsmSerial.available()) {
      char c = gsmSerial.read();
      res += c;
    }
    if (expect != NULL && strstr(res.c_str(), expect) != NULL) {
      return true;
    }
  }
  // Serial.print("AT Resp: "); Serial.println(res);
  return false;
}

// Initialize GSM module
bool initGSM() {
  // Basic checks
  if (!sendATexpect("AT", "OK", 2000)) return false;
  delay(200);
  if (!sendATexpect("ATE0", "OK", 2000)) { /* disable echo */ }
  delay(200);
  if (!sendATexpect("AT+CMGF=1", "OK", 2000)) return false; // set SMS text mode
  delay(200);
  // Auto attach GPRS disabled; keep default
  return true;
}

// Send SMS - blocks until message sent or timeout
bool sendSMS(const String &number, const String &message) {
  // Ensure text mode
  if (!sendATexpect("AT+CMGF=1", "OK", 2000)) return false;
  delay(200);
  String cmd = "AT+CMGS=\"" + number + "\"";
  gsmSerial.println(cmd);
  delay(500);
  // wait for '>' prompt (rough approach)
  unsigned long start = millis();
  bool prompt = false;
  String buf = "";
  while (millis() - start < 3000) {
    while (gsmSerial.available()) {
      char c = gsmSerial.read();
      buf += c;
      if (c == '>') { prompt = true; break; }
    }
    if (prompt) break;
  }
  if (!prompt) {
    // try to still send
  }
  gsmSerial.print(message);
  delay(500);
  gsmSerial.write(26); // Ctrl+Z to send
  // wait for +CMGS or OK
  start = millis();
  buf = "";
  while (millis() - start < 10000) {
    while (gsmSerial.available()) {
      char c = gsmSerial.read();
      buf += c;
    }
    if (strstr(buf.c_str(), "+CMGS") != NULL || strstr(buf.c_str(), "OK") != NULL) {
      return true;
    }
  }
  return false;
}

// Build message with GPS fix if available
String buildLocationMessage() {
  String msg = "Emergency! Need help.";
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    msg += " Location: ";
    msg += String(lat, 6);
    msg += ", ";
    msg += String(lng, 6);
    msg += " https://maps.google.com/?q=";
    msg += String(lat, 6);
    msg += ",";
    msg += String(lng, 6);
  } else {
    msg += " (GPS fix not available)";
  }
  return msg;
}

// Check for long press -> emergency SMS
void checkEmergencyButton() {
  bool pressed = (digitalRead(SWITCH_PIN) == LOW); // assume switch connects to GND when pressed (use pullup)
  if (pressed && !switchWasPressed) {
    switchPressedTime = millis();
    switchWasPressed = true;
  } else if (!pressed && switchWasPressed) {
    unsigned long held = millis() - switchPressedTime;
    switchWasPressed = false;
    if (held >= EMERGENCY_HOLD_MS) {
      // Trigger emergency
      beep(2, 150);
      vibrate(500);
      String msg = buildLocationMessage();
      bool ok = sendSMS(EMERGENCY_NUMBER, msg);
      if (ok) {
        // provide confirmation through buzzer/vibration
        beep(3, 100);
      } else {
        // failed
        for (int i=0;i<3;i++){ beep(1,200); delay(150);}
      }
    } else {
      // short press toggles power
      systemOn = !systemOn;
      if (systemOn) {
        beep(1,120);
      } else {
        beep(2,100);
        // turn off outputs
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(VIB_PIN, LOW);
        digitalWrite(FLASH_PIN, LOW);
      }
    }
  }
}

// Read GPS NMEA from gpsSerial and feed TinyGPS++
void feedGps() {
  // Make GPS serial the listening software serial when reading
  gpsSerial.listen();
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
}

// Allow a short period to listen to gsm serial for responses
void feedGsm() {
  gsmSerial.listen();
  while (gsmSerial.available()) {
    char c = gsmSerial.read(); // we discard here; used by functions above
    // Serial.print(c);
  }
}

// ---------------- SETUP ----------------
void setup() {
  // Serial for debug
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Smart Blind Stick - Starting..."));

  // pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(WATER_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VIB_PIN, OUTPUT);
  pinMode(FLASH_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP); // using internal pullup; pressed -> LOW
  pinMode(LDR_PIN, INPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(VIB_PIN, LOW);
  digitalWrite(FLASH_PIN, LOW);

  // start software serials
  gpsSerial.begin(9600);   // Many NEO-6M at 9600
  gsmSerial.begin(9600);   // SIM800L at 9600 typical

  // small delay for modules to init
  delay(1000);
  Serial.println(F("Initializing GSM module..."));
  bool gsmOk = initGSM();
  if (gsmOk) {
    Serial.println(F("GSM OK"));
  } else {
    Serial.println(F("GSM init failed or no response"));
  }

  // initial beep
  beep(2, 80);
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // Always feed GPS data (fast)
  // Switch gpsSerial to listen and read available bytes
  gpsSerial.listen();
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // small feed of gsm (to keep its buffer clear)
  gsmSerial.listen();
  while (gsmSerial.available()) {
    char c = gsmSerial.read(); // discard or you can parse if needed
    // Serial.write(c);
  }

  // Check switch short/long press actions
  checkEmergencyButton();

  // If system is OFF: skip sensor checks (but still parse GPS)
  if (!systemOn) {
    delay(150);
    return;
  }

  unsigned long now = millis();

  // 1) Ultrasonic obstacle detection
  if (now - lastUltrasonic >= ULTRASONIC_INTERVAL) {
    lastUltrasonic = now;
    long dist = readUltrasonicCM(TRIG_PIN, ECHO_PIN);
    if (dist > 0) {
      Serial.print(F("Distance: "));
      Serial.print(dist);
      Serial.println(F(" cm"));
      if (dist > 0 && dist <= DISTANCE_ALERT_CM) {
        // obstacle detected - alert pattern
        // Use buzzer + vibration
        for (int i=0;i<2;i++){
          digitalWrite(BUZZER_PIN, HIGH);
          digitalWrite(VIB_PIN, HIGH);
          delay(120);
          digitalWrite(BUZZER_PIN, LOW);
          digitalWrite(VIB_PIN, LOW);
          delay(80);
        }
      }
    } else {
      // no echo
      // Serial.println("Ultrasonic no echo");
    }
  }

  // 2) IR sensor (very close range)
  if (digitalRead(IR_PIN) == HIGH) {
    Serial.println(F("IR Obstacle detected"));
    // Short alert
    digitalWrite(BUZZER_PIN, HIGH);
    delay(120);
    digitalWrite(BUZZER_PIN, LOW);
    vibrate(120);
  }

  // 3) Water sensor (dangerous area)
  if (digitalRead(WATER_PIN) == HIGH) {
    Serial.println(F("Water detected!"));
    // persistent alert + send SMS alert (non-spam: one SMS per detection every ~30s)
    for (int i=0;i<3;i++) {
      beep(1, 180);
      vibrate(200);
      delay(150);
    }
    // send SMS with location
    String msg = "Warning: Water detected near user. ";
    msg += buildLocationMessage();
    bool ok = sendSMS(EMERGENCY_NUMBER, msg);
    if (ok) {
      Serial.println(F("Water alert SMS sent"));
    } else {
      Serial.println(F("Water alert SMS fail"));
    }
    delay(30000); // avoid spamming SMS; user can shorten/lengthen
  }

  // 4) LDR -> Torch automatic
  int ldrVal = analogRead(LDR_PIN);
  Serial.print(F("LDR: "));
  Serial.println(ldrVal);
  if (ldrVal < LDR_THRESHOLD) {
    // dark -> turn torch ON
    digitalWrite(FLASH_PIN, HIGH);
  } else {
    digitalWrite(FLASH_PIN, LOW);
  }

  // 5) GPS occasional debug
  if (gps.location.isValid()) {
    Serial.print(F("GPS lat: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" lng: "));
    Serial.println(gps.location.lng(), 6);
  } else {
    // Serial.println("GPS no fix");
  }

  // small delay to avoid busy-loop
  delay(80);
}
