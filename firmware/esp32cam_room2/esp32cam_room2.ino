/*
========================================================
  SmartNest — Room 2 Controller
  Board   : AI Thinker ESP32-CAM
  Sensors : DHT11, MQ2, Flame, PIR, Rain, LDR
  Output  : 3 Relays, Buzzer, Camera Stream
  Comms   : WiFi → serves sensor JSON on port 80
            Camera MJPEG stream on port 81
========================================================

  ⚠️  ESP32-CAM HAS VERY FEW FREE PINS.
      Most GPIO are used by the camera module.
      Only use the pins listed below.

  PIN WIRING (ESP32-CAM → Component)
  ────────────────────────────────────────────────────
  GPIO  2  → DHT11 DATA                (10kΩ pull-up to 3.3V)
  GPIO  3  → PIR OUT                   (3.3V)
  GPIO 16  → Flame Sensor DOUT         (active LOW)
  GPIO 13  → Relay 1 IN  (Light)       (Active LOW)
  GPIO 12  → Relay 2 IN  (Fan)         (Active LOW)
  GPIO 14  → Relay 3 IN  (Outlet)      (Active LOW)
  GPIO 15  → Buzzer +                  (via NPN transistor to 5V)
  GPIO  4  → Flash LED (night vision)  (built-in on ESP32-CAM)

  ⚠️  ESP32-CAM ADC2 pins (0,2,4,12-15,25-27) don't work
      during WiFi. So MQ2 analog, Rain, LDR CANNOT be
      read accurately while WiFi is on.
      SOLUTION:  Use digital threshold outputs of MQ2 & rain
                 sensors (DOUT pin) instead of analog.

  GPIO  5  → MQ2 DOUT (digital)        (HIGH = gas detected)
  GPIO  1  → Rain Sensor DOUT          (LOW  = rain detected)
             (GPIO1 is TX — disconnect for upload, reconnect after)

  ── CAMERA ────────────────────────────────────────
  All camera pins are internal (ribbon cable).
  Board: AI Thinker ESP32-CAM (select in Arduino IDE)

  ── POWER ─────────────────────────────────────────
  5V → 5V pin (needs 5V 2A+ supply — camera draws 300mA)
  GND → GND
  ────────────────────────────────────────────────────
  Upload method:
   1. Connect IO0 → GND before powering
   2. Power up → click Upload in Arduino IDE
   3. After "Connecting..." appears, press RST button once
   4. After upload: disconnect IO0 → GND, press RST again
========================================================
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "secrets.h"      // WiFi name/password (not uploaded to GitHub)
#include <WebServer.h>
#include <DHT.h>

// ── WiFi ─────────────────────────────────────────
const char* SSID = WIFI_SSID;   // set in secrets.h
const char* PASS = WIFI_PASS;   // set in secrets.h

// ── Camera Pins (AI Thinker) ──────────────────────
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// ── Sensor Pins ───────────────────────────────────
#define DHT_PIN       2
#define PIR_PIN       3
#define FLAME_PIN    16
#define MQ2_DOUT      5    // Digital output of MQ2 (use DOUT not AOUT)
#define RAIN_DOUT     1    // Digital output of rain sensor (TX pin — remove for upload)

// ── Actuator Pins ─────────────────────────────────
#define RELAY1       13
#define RELAY2       12
#define RELAY3       14
#define BUZZER       15
#define FLASH_LED     4

// ── Thresholds ────────────────────────────────────
#define GAS_THRESHOLD_DOUT  LOW    // MQ2 DOUT goes LOW when gas detected (most modules)

// ── Objects ──────────────────────────────────────
DHT       dht(DHT_PIN, DHT11);
WebServer server(80);
WebServer camServer(81);   // Camera stream on separate port

// ── State ────────────────────────────────────────
float  temp = 0, hum = 0;
bool   gasDetected = false, flame = false, motion = false, rainDetected = false;
bool   r1 = false, r2 = false, r3 = false, buzzer = false, flash = false;
String lastAlert = "";
unsigned long lastRead = 0;

// ── CORS helper ───────────────────────────────────
void addCORS(WebServer& s) {
  s.sendHeader("Access-Control-Allow-Origin", "*");
  s.sendHeader("Access-Control-Allow-Methods", "GET,POST");
  s.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ── /data JSON ────────────────────────────────────
void handleData() {
  addCORS(server);
  String s = "{";
  s += "\"temp\":"  + String(temp,1)         + ",";
  s += "\"hum\":"   + String(hum,1)          + ",";
  s += "\"gas\":"   + String(gasDetected)    + ",";
  s += "\"flame\":" + String(flame)          + ",";
  s += "\"motion\":" + String(motion)        + ",";
  s += "\"rain\":"  + String(rainDetected)   + ",";
  s += "\"r1\":"    + String(r1)             + ",";
  s += "\"r2\":"    + String(r2)             + ",";
  s += "\"r3\":"    + String(r3)             + ",";
  s += "\"buzzer\":" + String(buzzer)        + ",";
  s += "\"flash\":" + String(flash)          + ",";
  s += "\"alert\":\"" + lastAlert           + "\"";
  s += "}";
  server.send(200, "application/json", s);
}

// ── /cmd POST ────────────────────────────────────
void handleCmd() {
  addCORS(server);
  if (!server.hasArg("plain")) { server.send(400,"text/plain","No body"); return; }
  String b = server.arg("plain");
  if (b.indexOf("\"r1\":1") >= 0) { r1=true;  setRelay(RELAY1,true);  }
  if (b.indexOf("\"r1\":0") >= 0) { r1=false; setRelay(RELAY1,false); }
  if (b.indexOf("\"r2\":1") >= 0) { r2=true;  setRelay(RELAY2,true);  }
  if (b.indexOf("\"r2\":0") >= 0) { r2=false; setRelay(RELAY2,false); }
  if (b.indexOf("\"r3\":1") >= 0) { r3=true;  setRelay(RELAY3,true);  }
  if (b.indexOf("\"r3\":0") >= 0) { r3=false; setRelay(RELAY3,false); }
  if (b.indexOf("\"bz\":1") >= 0) { buzzer=true;  digitalWrite(BUZZER,HIGH); }
  if (b.indexOf("\"bz\":0") >= 0) { buzzer=false; digitalWrite(BUZZER,LOW);  }
  if (b.indexOf("\"flash\":1") >= 0) { flash=true;  digitalWrite(FLASH_LED,HIGH); }
  if (b.indexOf("\"flash\":0") >= 0) { flash=false; digitalWrite(FLASH_LED,LOW);  }
  server.send(200,"application/json","{\"ok\":1}");
}

void handleOPT() { addCORS(server); server.send(204); }

// ── Camera stream (MJPEG) ─────────────────────────
void handleStream() {
  WiFiClient client = camServer.client();
  String res = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  camServer.sendContent(res);
  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) break;
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
  }
}

void handleCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { camServer.send(500,"text/plain","Cam fail"); return; }
  camServer.sendHeader("Content-Disposition","inline; filename=snap.jpg");
  camServer.send_P(200,"image/jpeg",(const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ── Relay helper ─────────────────────────────────
void setRelay(int pin, bool on) { digitalWrite(pin, on ? LOW : HIGH); }

// ── Read sensors ─────────────────────────────────
void readSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temp = t;
  if (!isnan(h)) hum  = h;

  // Using DIGITAL outputs only (no ADC2 conflict)
  gasDetected  = (digitalRead(MQ2_DOUT)  == LOW);  // LOW = gas detected
  flame        = (digitalRead(FLAME_PIN) == LOW);   // LOW = flame detected
  motion       = (digitalRead(PIR_PIN)   == HIGH);
  rainDetected = (digitalRead(RAIN_DOUT) == LOW);   // LOW = rain detected
}

// ── Auto rules ────────────────────────────────────
void autoRules() {
  if (gasDetected && !r2) {
    r2 = true; setRelay(RELAY2, true);
    lastAlert = "Fan auto-ON: gas detected";
  }
  if (flame) {
    if (!buzzer) { buzzer=true; digitalWrite(BUZZER,HIGH); }
    lastAlert = "FLAME DETECTED";
  }
  if (motion) {
    lastAlert = "Motion detected";
  }
}

// ── Camera init ───────────────────────────────────
void initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0 = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2 = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4 = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6 = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM; cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM; cfg.pin_href = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM; cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn  = PWDN_GPIO_NUM;  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_RGB565;
  if (psramFound()) {
    cfg.frame_size   = FRAMESIZE_VGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
  } else {
    cfg.frame_size   = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 15;
    cfg.fb_count     = 1;
  }
  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED: 0x%x\n", err);
  } else {
    Serial.println("Camera OK");
    sensor_t* s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  }
}

// ── SETUP ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Init camera FIRST before WiFi
  initCamera();

  pinMode(FLAME_PIN,  INPUT);
  pinMode(PIR_PIN,    INPUT);
  pinMode(MQ2_DOUT,   INPUT);
  pinMode(RAIN_DOUT,  INPUT);
  pinMode(RELAY1,     OUTPUT); digitalWrite(RELAY1, HIGH);
  pinMode(RELAY2,     OUTPUT); digitalWrite(RELAY2, HIGH);
  pinMode(RELAY3,     OUTPUT); digitalWrite(RELAY3, HIGH);
  pinMode(BUZZER,     OUTPUT); digitalWrite(BUZZER,  LOW);
  pinMode(FLASH_LED,  OUTPUT); digitalWrite(FLASH_LED,LOW);

  dht.begin();

  WiFi.begin(SSID, PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nRoom 2 IP:      " + WiFi.localIP().toString());
  Serial.println("Camera stream:  http://" + WiFi.localIP().toString() + ":81/stream");
  Serial.println("Snapshot:       http://" + WiFi.localIP().toString() + ":81/capture");

  server.on("/data",  HTTP_GET,     handleData);
  server.on("/cmd",   HTTP_POST,    handleCmd);
  server.on("/cmd",   HTTP_OPTIONS, handleOPT);
  server.begin();

  camServer.on("/stream",  handleStream);
  camServer.on("/capture", handleCapture);
  camServer.begin();
}

// ── LOOP ──────────────────────────────────────────
void loop() {
  server.handleClient();
  camServer.handleClient();

  unsigned long now = millis();
  if (now - lastRead >= 2000) {
    lastRead = now;
    readSensors();
    autoRules();
  }
}
