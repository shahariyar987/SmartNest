/*
 ============================================================
  SmartNest - Room 2 Controller (ESP32-CAM)
  Camera  : RHYX M21-45 (RGB565 format)
  Board   : AI Thinker ESP32-CAM

  HOW TO UPLOAD:
  1. Tools -> Board      -> "AI Thinker ESP32-CAM"
  2. Tools -> Partition  -> "Huge APP (3MB No OTA/1MB SPIFFS)"
  3. Click Upload
  4. When "Connecting....___" appears -> press RST button once
  5. After upload done -> open Serial Monitor 115200 baud
  6. Press RST again -> you will see the IP address

  PIN WIRING:
  GPIO  2  -> DHT11 DATA         (10k pull-up to 3.3V)
  GPIO 16  -> Flame sensor DOUT  (Active LOW)
  GPIO  3  -> PIR motion OUT
  GPIO  1  -> Rain sensor DOUT   (disconnect during upload!)
  GPIO  5  -> MQ2 DOUT           (digital only)
  GPIO 13  -> Relay 1 (Light)    (Active LOW)
  GPIO 12  -> Relay 2 (Fan)      (Active LOW)
  GPIO 14  -> Relay 3 (Outlet)   (Active LOW)
  GPIO 15  -> Buzzer             (via BC547 transistor)
  GPIO  4  -> Flash LED          (built-in)

  POWER:
  5V pad on ESP32-CAM board -> phone charger 5V (2A+)
  GND -> common ground with all sensors and 12V adapter GND
 ============================================================
*/

#include "esp_camera.h"
#include "img_converters.h"   // frame2jpg() - converts RGB565 -> JPEG for streaming
#include <WiFi.h>
#include "secrets.h"      // WiFi name/password (not uploaded to GitHub)
#include <WebServer.h>
#include <DHT.h>

// -------------------------------------------------------
//  WiFi settings: copy secrets.example.h to secrets.h and edit it
// -------------------------------------------------------
const char* SSID = WIFI_SSID;   // set in secrets.h
const char* PASS = WIFI_PASS;   // set in secrets.h

// -------------------------------------------------------
//  AI Thinker ESP32-CAM fixed camera pins - DO NOT change
// -------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// -------------------------------------------------------
//  Your sensor and actuator pins
// -------------------------------------------------------
#define DHT_PIN    2
#define FLAME_PIN 16
#define PIR_PIN    3
#define MQ2_PIN    5   // DOUT only - no analog on ESP32-CAM with WiFi
#define RAIN_PIN   1   // DOUT only - disconnect this wire during upload!

#define RELAY1    13   // Light    - Active LOW relay
#define RELAY2    12   // Fan      - Active LOW relay
#define RELAY3    14   // Outlet   - Active LOW relay
#define BUZZER    15   // via BC547 NPN transistor
#define FLASH_LED  4   // built-in flash LED on ESP32-CAM board

// -------------------------------------------------------
//  Objects
// -------------------------------------------------------
DHT       dht(DHT_PIN, DHT11);
WebServer server(80);     // sensor data + relay control
WebServer camSrv(81);     // camera stream

// -------------------------------------------------------
//  Global state
// -------------------------------------------------------
float  temp     = 0.0;
float  hum      = 0.0;
bool   gas      = false;
bool   flame    = false;
bool   motion   = false;
bool   rain     = false;
bool   r1       = false;
bool   r2       = false;
bool   r3       = false;
bool   buzzerOn = false;
bool   flashOn  = false;
String alertMsg = "";

unsigned long lastRead = 0;

// =======================================================
//  CAMERA INIT - RGB565 mode with RAM-safe frame sizes
// =======================================================
bool initCamera() {
  camera_config_t cfg;

  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;

  // RGB565 pixel format (what RHYX M21-45 supports)
  cfg.pixel_format = PIXFORMAT_RGB565;
  cfg.xclk_freq_hz = 10000000;          // 10MHz - more stable than 20MHz
  cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  cfg.fb_count     = 1;                 // single frame buffer - safest
  cfg.jpeg_quality = 12;               // quality used after RGB->JPEG conversion

  // ---------------------------------------------------
  // IMPORTANT: RGB565 RAM requirements per frame:
  //   96 x 96  =  18 KB  -> fits in internal DRAM (no PSRAM needed)
  //   160x120  =  38 KB  -> needs PSRAM
  //   320x240  = 150 KB  -> needs PSRAM  (THIS WAS CRASHING YOU)
  // ---------------------------------------------------
  if (psramFound()) {
    Serial.println("PSRAM found - using QQVGA 160x120");
    cfg.frame_size  = FRAMESIZE_QQVGA;   // 160x120 -> 38KB fits in PSRAM
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("No PSRAM - using 96x96 (18KB fits in DRAM)");
    cfg.frame_size  = FRAMESIZE_96X96;   // 96x96 -> 18KB fits in DRAM
    cfg.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&cfg);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED: 0x%x\n", err);
    Serial.println("Check: ribbon cable fully inserted? Metal contacts facing DOWN?");
    return false;
  }

  // Fine-tune the sensor image
  sensor_t* s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_vflip(s, 1);          // flip vertical   (remove if image looks correct)
    s->set_hmirror(s, 1);        // flip horizontal  (remove if image looks correct)
    s->set_brightness(s, 1);     // slightly brighter
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);       // auto white balance
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);  // auto exposure
  }

  Serial.println("Camera OK (RGB565 mode)");
  return true;
}

// =======================================================
//  MJPEG STREAM - /stream on port 81
//  Grabs RGB565 frame -> converts to JPEG -> sends multipart
// =======================================================
void handleStream() {
  WiFiClient client = camSrv.client();
  Serial.println("Stream: client connected");

  // Send MJPEG multipart header
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Cache-Control: no-cache, no-store, must-revalidate");
  client.println("Pragma: no-cache");
  client.println();

  uint8_t* jpegBuf = NULL;
  size_t   jpegLen = 0;

  while (client.connected()) {

    // 1. Grab raw RGB565 frame from camera
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Stream: frame grab failed");
      delay(100);
      continue;
    }

    // 2. Convert RGB565 frame to JPEG
    //    frame2jpg() allocates jpegBuf - we must free() it after use
    bool converted = frame2jpg(fb, 80, &jpegBuf, &jpegLen);
    esp_camera_fb_return(fb);   // return frame buffer immediately after conversion

    if (!converted || jpegBuf == NULL) {
      Serial.println("Stream: RGB565->JPEG conversion failed");
      delay(100);
      continue;
    }

    // 3. Send the JPEG as one multipart frame
    client.print("--frame\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.printf("Content-Length: %u\r\n\r\n", jpegLen);
    client.write(jpegBuf, jpegLen);
    client.print("\r\n");

    // 4. Free the JPEG buffer - important, or memory leaks
    free(jpegBuf);
    jpegBuf = NULL;
    jpegLen = 0;

    delay(100);  // ~10 fps
  }

  Serial.println("Stream: client disconnected");
}

// =======================================================
//  SINGLE SNAPSHOT - /capture on port 81
// =======================================================
void handleCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    camSrv.send(500, "text/plain", "Capture failed - camera not ready");
    return;
  }

  uint8_t* jpegBuf = NULL;
  size_t   jpegLen = 0;
  bool converted = frame2jpg(fb, 80, &jpegBuf, &jpegLen);
  esp_camera_fb_return(fb);

  if (!converted || jpegBuf == NULL) {
    camSrv.send(500, "text/plain", "RGB565 to JPEG conversion failed");
    return;
  }

  camSrv.sendHeader("Content-Disposition", "inline; filename=snapshot.jpg");
  camSrv.sendHeader("Access-Control-Allow-Origin", "*");
  camSrv.send_P(200, "image/jpeg", (const char*)jpegBuf, jpegLen);
  free(jpegBuf);
}

// =======================================================
//  SENSOR DATA - GET /data  returns JSON
// =======================================================
void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String j = "{";
  j += "\"temp\":"    + String(temp, 1)         + ",";
  j += "\"hum\":"     + String(hum,  1)         + ",";
  j += "\"gas\":"     + String(gas    ? 1 : 0)  + ",";
  j += "\"flame\":"   + String(flame  ? 1 : 0)  + ",";
  j += "\"motion\":"  + String(motion ? 1 : 0)  + ",";
  j += "\"rain\":"    + String(rain   ? 1 : 0)  + ",";
  j += "\"r1\":"      + String(r1     ? 1 : 0)  + ",";
  j += "\"r2\":"      + String(r2     ? 1 : 0)  + ",";
  j += "\"r3\":"      + String(r3     ? 1 : 0)  + ",";
  j += "\"buzzer\":"  + String(buzzerOn ? 1 : 0) + ",";
  j += "\"flash\":"   + String(flashOn  ? 1 : 0) + ",";
  j += "\"alert\":\"" + alertMsg + "\"";
  j += "}";
  server.send(200, "application/json", j);
}

// =======================================================
//  CONTROL COMMANDS - POST /cmd  body: {"r1":1} etc.
// =======================================================
void handleCmd() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No body");
    return;
  }
  String b = server.arg("plain");

  if (b.indexOf("\"r1\":1") >= 0) { r1 = true;       setRelay(RELAY1, true);          }
  if (b.indexOf("\"r1\":0") >= 0) { r1 = false;      setRelay(RELAY1, false);         }
  if (b.indexOf("\"r2\":1") >= 0) { r2 = true;       setRelay(RELAY2, true);          }
  if (b.indexOf("\"r2\":0") >= 0) { r2 = false;      setRelay(RELAY2, false);         }
  if (b.indexOf("\"r3\":1") >= 0) { r3 = true;       setRelay(RELAY3, true);          }
  if (b.indexOf("\"r3\":0") >= 0) { r3 = false;      setRelay(RELAY3, false);         }
  if (b.indexOf("\"bz\":1") >= 0) { buzzerOn = true;  digitalWrite(BUZZER, HIGH);      }
  if (b.indexOf("\"bz\":0") >= 0) { buzzerOn = false; digitalWrite(BUZZER, LOW);       }
  if (b.indexOf("\"flash\":1") >= 0) { flashOn = true;  digitalWrite(FLASH_LED, HIGH); }
  if (b.indexOf("\"flash\":0") >= 0) { flashOn = false; digitalWrite(FLASH_LED, LOW);  }

  server.send(200, "application/json", "{\"ok\":1}");
}

void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.send(204);
}

// =======================================================
//  RELAY HELPER - most relay modules are Active LOW
//  Active LOW: digitalWrite LOW = relay ON, HIGH = relay OFF
// =======================================================
void setRelay(int pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

// =======================================================
//  READ SENSORS
// =======================================================
void readSensors() {
  // DHT11 temperature and humidity
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) temp = t;
  if (!isnan(h)) hum  = h;

  // Digital sensors (DOUT pin only - no ADC2 analog while WiFi is on)
  flame  = (digitalRead(FLAME_PIN) == LOW);   // active LOW - LOW means flame
  motion = (digitalRead(PIR_PIN)   == HIGH);  // HIGH means motion detected
  gas    = (digitalRead(MQ2_PIN)   == LOW);   // LOW means gas detected
  rain   = (digitalRead(RAIN_PIN)  == LOW);   // LOW means rain detected
}

// =======================================================
//  AUTO RULES
// =======================================================
void autoRules() {
  // Fan auto-ON when gas detected
  if (gas && !r2) {
    r2 = true;
    setRelay(RELAY2, true);
    alertMsg = "Fan auto-ON: gas detected";
    Serial.println("[AUTO] Fan ON - gas detected");
  }

  // Buzzer auto-ON when flame detected
  if (flame && !buzzerOn) {
    buzzerOn = true;
    digitalWrite(BUZZER, HIGH);
    alertMsg = "FLAME DETECTED!";
    Serial.println("[ALERT] Flame detected!");
  }

  // Motion alert
  if (motion) {
    alertMsg = "Motion detected";
  }

  // Clear alert when all sensors normal
  if (!gas && !flame && !motion) {
    alertMsg = "";
  }
}

// =======================================================
//  SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== SmartNest ESP32-CAM Room 2 (RGB565) ===");

  // IMPORTANT: init camera BEFORE WiFi starts
  // WiFi uses a lot of RAM and can prevent camera from getting buffer space
  Serial.println("Starting camera...");
  bool camOK = initCamera();

  // Setup pins
  pinMode(FLAME_PIN, INPUT);
  pinMode(PIR_PIN,   INPUT);
  pinMode(MQ2_PIN,   INPUT);
  pinMode(RAIN_PIN,  INPUT);

  pinMode(RELAY1,    OUTPUT); setRelay(RELAY1, false);   // start OFF
  pinMode(RELAY2,    OUTPUT); setRelay(RELAY2, false);
  pinMode(RELAY3,    OUTPUT); setRelay(RELAY3, false);
  pinMode(BUZZER,    OUTPUT); digitalWrite(BUZZER,    LOW);
  pinMode(FLASH_LED, OUTPUT); digitalWrite(FLASH_LED, LOW);

  dht.begin();

  // Connect WiFi
  Serial.printf("Connecting to WiFi: %s ", SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n-----------------------------------------");
    Serial.println("WiFi connected!");
    Serial.print("Sensor data  : http://");
    Serial.print(WiFi.localIP());
    Serial.println("/data");
    Serial.print("Control      : http://");
    Serial.print(WiFi.localIP());
    Serial.println("/cmd");
    if (camOK) {
      Serial.print("Live stream  : http://");
      Serial.print(WiFi.localIP());
      Serial.println(":81/stream");
      Serial.print("Snapshot     : http://");
      Serial.print(WiFi.localIP());
      Serial.println(":81/capture");
    } else {
      Serial.println("Camera FAILED - stream not available");
      Serial.println("Sensors and relay control still working.");
    }
    Serial.println("-----------------------------------------");
  } else {
    Serial.println("\nWiFi FAILED - check your SSID and password");
  }

  // Start sensor/control server on port 80
  server.on("/data",    HTTP_GET,     handleData);
  server.on("/cmd",     HTTP_POST,    handleCmd);
  server.on("/cmd",     HTTP_OPTIONS, handleOptions);
  server.begin();
  Serial.println("Sensor server started on port 80");

  // Start camera server on port 81 (only if camera init succeeded)
  if (camOK) {
    camSrv.on("/stream",  HTTP_GET, handleStream);
    camSrv.on("/capture", HTTP_GET, handleCapture);
    camSrv.begin();
    Serial.println("Camera server started on port 81");
  }

  Serial.println("=== Ready ===\n");
}

// =======================================================
//  LOOP
// =======================================================
void loop() {
  // Handle incoming HTTP requests for both servers
  server.handleClient();
  camSrv.handleClient();

  // Read sensors every 2 seconds
  unsigned long now = millis();
  if (now - lastRead >= 2000) {
    lastRead = now;
    readSensors();
    autoRules();
  }
}
