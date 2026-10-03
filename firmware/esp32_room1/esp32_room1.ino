/*
========================================================
  SmartNest — Room 1 Controller
  Board   : ESP32 Dev Module
  Sensors : DHT11, MQ2, Flame, PIR, Rain, LDR
  Security: NFC(MFRC522), Door Lock, Laser, Sonar
  Output  : 3 Relays, Buzzer
  Comms   : WiFi → serves sensor data as JSON on port 80
            Receives commands via HTTP POST
========================================================

  PIN WIRING (ESP32 → Component)
  ────────────────────────────────────────────────────
  GPIO  4  → DHT11 DATA pin          (10kΩ pull-up to 3.3V)
  GPIO 34  → MQ2 AOUT (analog)       (3.3V power!)
  GPIO 35  → Flame Sensor DOUT       (3.3V, active LOW)
  GPIO 32  → PIR OUT                 (3.3V)
  GPIO 33  → Rain Sensor AOUT        (3.3V)
  GPIO 36  → LDR middle of divider   (LDR+10kΩ divider)
  GPIO  5  → Sonar TRIG              (HC-SR04)
  GPIO 18  → Sonar ECHO              (HC-SR04, 3.3V tolerant)
  GPIO 39  → Laser receiver LDR      (same divider setup as LDR)
  GPIO 21  → NFC SS/SDA              (MFRC522 - SPI)
  GPIO 22  → NFC RST                 (MFRC522)
  GPIO 23  → NFC MOSI                (MFRC522 - SPI MOSI)
  GPIO 19  → NFC MISO                (MFRC522 - SPI MISO)
  GPIO 18  → NFC SCK                 (SPI SCK, shared with Sonar)
  GPIO 26  → Relay 1 IN  (Light)     (Active LOW)
  GPIO 27  → Relay 2 IN  (Fan)       (Active LOW)
  GPIO 14  → Relay 3 IN  (Outlet)    (Active LOW)
  GPIO 25  → Buzzer +                (5V buzzer via NPN transistor)
  GPIO 13  → Door Lock Relay IN      (12V lock, flyback diode!)
  GPIO 15  → Laser Module Signal     (5mW laser)
  ────────────────────────────────────────────────────
  POWER:
   MQ2       → 5V (needs 5V heater), AOUT → 3.3V tolerant divider
   DHT11     → 3.3V
   MFRC522   → 3.3V only!
   HC-SR04   → 5V, ECHO → 1kΩ+2kΩ divider to 3.3V
   Relays    → 5V VCC, IN → 3.3V GPIO (most work)
   Door Lock → External 12V supply through relay
========================================================
*/

#include <WiFi.h>
#include "secrets.h"      // WiFi name/password (not uploaded to GitHub)
#include <WebServer.h>
#include <DHT.h>
#include <SPI.h>
#include <MFRC522.h>

// ── WiFi ─────────────────────────────────────────
const char* SSID = WIFI_SSID;   // set in secrets.h
const char* PASS = WIFI_PASS;   // set in secrets.h

// ── Pins ─────────────────────────────────────────
#define DHT_PIN      4
#define MQ2_PIN     34
#define FLAME_PIN   35
#define PIR_PIN     32
#define RAIN_PIN    33
#define LDR_PIN     36
#define TRIG_PIN     5
#define ECHO_PIN    18   // Use voltage divider (5V→3.3V)!
#define LASER_LDR   39
#define NFC_SS      21
#define NFC_RST     22
#define RELAY1      26   // Light
#define RELAY2      27   // Fan
#define RELAY3      14   // Outlet
#define BUZZER      25
#define DOOR_LOCK   13
#define LASER_OUT   15

// ── Thresholds ───────────────────────────────────
#define GAS_THRESHOLD   400   // ppm — auto-fan above this
#define RAIN_THRESHOLD  500   // analog — rain if below this
#define LASER_THRESHOLD 300   // LDR ADC — broken if below this
#define SONAR_ALARM     0.5   // metres — alert if closer

// ── Authorized NFC UIDs ─────────────────────────
// To find your card UID: open Serial Monitor after upload, scan card
String AUTH_CARDS[] = {
  "A3 F2 D1 C4",  // Card 1 — replace with your UID
  "B9 D1 22 88",  // Card 2
};
const int NUM_CARDS = 2;

// ── Objects ──────────────────────────────────────
DHT       dht(DHT_PIN, DHT11);
MFRC522   nfc(NFC_SS, NFC_RST);
WebServer server(80);

// ── State ────────────────────────────────────────
float   temp = 0, hum = 0;
int     gas = 0, rain = 0, light = 0, laserADC = 0;
bool    flame = false, motion = false, laserBroken = false, doorLocked = true;
float   dist = 0;
bool    r1 = false, r2 = false, r3 = false, buzzer = false;
String  lastAlert = "";
String  lastNFC   = "";
unsigned long doorOpenAt = 0;
unsigned long lastRead   = 0;

// ── CORS header helper ───────────────────────────
void addCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ── /data → returns JSON of all sensor values ────
void handleData() {
  addCORS();
  String rain_pct = String(map(rain, 4095, 0, 0, 100));
  String light_pct= String(map(light,0,4095,0,100));
  String s = "{";
  s += "\"temp\":"    + String(temp,1)   + ",";
  s += "\"hum\":"     + String(hum,1)    + ",";
  s += "\"gas\":"     + String(gas)      + ",";
  s += "\"flame\":"   + String(flame)    + ",";
  s += "\"motion\":"  + String(motion)   + ",";
  s += "\"rain\":"    + String(rain)     + ",";
  s += "\"rain_pct\":" + rain_pct        + ",";
  s += "\"light\":"   + String(light)    + ",";
  s += "\"light_pct\":" + light_pct      + ",";
  s += "\"dist\":"    + String(dist,2)   + ",";
  s += "\"laser\":"   + String(!laserBroken) + ",";
  s += "\"door\":"    + String(!doorLocked)  + ",";
  s += "\"r1\":"      + String(r1)       + ",";
  s += "\"r2\":"      + String(r2)       + ",";
  s += "\"r3\":"      + String(r3)       + ",";
  s += "\"buzzer\":"  + String(buzzer)   + ",";
  s += "\"alert\":\""  + lastAlert       + "\",";
  s += "\"nfc\":\""   + lastNFC         + "\"";
  s += "}";
  server.send(200, "application/json", s);
}

// ── /cmd → receives control commands ─────────────
void handleCmd() {
  addCORS();
  if (!server.hasArg("plain")) { server.send(400,"text/plain","No body"); return; }
  String body = server.arg("plain");

  if (body.indexOf("\"r1\":1") >= 0) { r1=true;  setRelay(RELAY1,true);  }
  if (body.indexOf("\"r1\":0") >= 0) { r1=false; setRelay(RELAY1,false); }
  if (body.indexOf("\"r2\":1") >= 0) { r2=true;  setRelay(RELAY2,true);  }
  if (body.indexOf("\"r2\":0") >= 0) { r2=false; setRelay(RELAY2,false); }
  if (body.indexOf("\"r3\":1") >= 0) { r3=true;  setRelay(RELAY3,true);  }
  if (body.indexOf("\"r3\":0") >= 0) { r3=false; setRelay(RELAY3,false); }
  if (body.indexOf("\"bz\":1") >= 0) { buzzer=true;  digitalWrite(BUZZER,HIGH); }
  if (body.indexOf("\"bz\":0") >= 0) { buzzer=false; digitalWrite(BUZZER,LOW);  }
  if (body.indexOf("\"door\":1") >= 0) {
    doorLocked=false; doorOpenAt=millis();
    digitalWrite(DOOR_LOCK,LOW);   // LOW = unlock for most relay modules
  }
  if (body.indexOf("\"door\":0") >= 0) {
    doorLocked=true;
    digitalWrite(DOOR_LOCK,HIGH);
  }
  server.send(200,"application/json","{\"ok\":1}");
}

void handleOPTIONS() { addCORS(); server.send(204); }

// ── Relay helper (active LOW) ─────────────────────
void setRelay(int pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

// ── Sonar ─────────────────────────────────────────
float readSonar() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  if (dur == 0) return 9.99;
  return dur * 0.034 / 2.0 / 100.0;
}

// ── NFC ───────────────────────────────────────────
void checkNFC() {
  if (!nfc.PICC_IsNewCardPresent()) return;
  if (!nfc.PICC_ReadCardSerial())   return;
  String uid = "";
  for (byte i = 0; i < nfc.uid.size; i++) {
    if (nfc.uid.uidByte[i] < 0x10) uid += "0";
    uid += String(nfc.uid.uidByte[i], HEX);
    if (i < nfc.uid.size-1) uid += " ";
  }
  uid.toUpperCase();
  Serial.print("[NFC] Card: "); Serial.println(uid);

  bool auth = false;
  for (int i = 0; i < NUM_CARDS; i++) {
    if (uid == AUTH_CARDS[i]) { auth = true; break; }
  }

  if (auth) {
    lastNFC = "granted:" + uid;
    doorLocked = false;
    doorOpenAt = millis();
    digitalWrite(DOOR_LOCK, LOW);
    Serial.println("[NFC] Access GRANTED");
  } else {
    lastNFC = "denied:" + uid;
    // Short alarm beep
    digitalWrite(BUZZER, HIGH); delay(300); digitalWrite(BUZZER, LOW);
    Serial.println("[NFC] Access DENIED");
  }
  nfc.PICC_HaltA();
  nfc.PCD_StopCrypto1();
}

// ── Sensors ───────────────────────────────────────
void readSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temp = t;
  if (!isnan(h)) hum  = h;

  gas        = analogRead(MQ2_PIN);
  flame      = (digitalRead(FLAME_PIN) == LOW);
  motion     = (digitalRead(PIR_PIN)   == HIGH);
  rain       = analogRead(RAIN_PIN);
  light      = analogRead(LDR_PIN);
  laserADC   = analogRead(LASER_LDR);
  laserBroken= (laserADC < LASER_THRESHOLD);
  dist       = readSonar();
}

// ── Auto-rules ────────────────────────────────────
void autoRules() {
  // Fan on if gas high
  if (gas > GAS_THRESHOLD && !r2) {
    r2 = true; setRelay(RELAY2, true);
    lastAlert = "Fan auto-ON: gas high";
  }
  // Alarm if flame
  if (flame) {
    if (!buzzer) { buzzer=true; digitalWrite(BUZZER,HIGH); }
    lastAlert = "FLAME DETECTED";
  }
  // Alarm if laser broken
  if (laserBroken) {
    if (!buzzer) { buzzer=true; digitalWrite(BUZZER,HIGH); }
    lastAlert = "Laser tripwire broken";
  }
  // Alarm if too close (sonar)
  if (dist < SONAR_ALARM && dist > 0.02) {
    lastAlert = "Intruder close: " + String(dist,2) + "m";
  }
}

// ── SETUP ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(FLAME_PIN,  INPUT);
  pinMode(PIR_PIN,    INPUT);
  pinMode(TRIG_PIN,   OUTPUT);
  pinMode(ECHO_PIN,   INPUT);
  pinMode(RELAY1,     OUTPUT); digitalWrite(RELAY1, HIGH);
  pinMode(RELAY2,     OUTPUT); digitalWrite(RELAY2, HIGH);
  pinMode(RELAY3,     OUTPUT); digitalWrite(RELAY3, HIGH);
  pinMode(BUZZER,     OUTPUT); digitalWrite(BUZZER,  LOW);
  pinMode(DOOR_LOCK,  OUTPUT); digitalWrite(DOOR_LOCK,HIGH);
  pinMode(LASER_OUT,  OUTPUT); digitalWrite(LASER_OUT,HIGH);

  dht.begin();
  SPI.begin();
  nfc.PCD_Init();

  WiFi.begin(SSID, PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nRoom 1 IP: " + WiFi.localIP().toString());

  server.on("/data",    HTTP_GET,  handleData);
  server.on("/cmd",     HTTP_POST, handleCmd);
  server.on("/cmd",     HTTP_OPTIONS, handleOPTIONS);
  server.begin();
}

// ── LOOP ──────────────────────────────────────────
void loop() {
  server.handleClient();

  unsigned long now = millis();
  if (now - lastRead >= 2000) {
    lastRead = now;
    readSensors();
    autoRules();
    checkNFC();
  }

  // Auto-lock door after 5 seconds
  if (!doorLocked && (millis() - doorOpenAt >= 5000)) {
    doorLocked = true;
    digitalWrite(DOOR_LOCK, HIGH);
  }
}
