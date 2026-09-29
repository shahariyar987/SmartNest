# SmartNest — IoT Smart Home System

An ESP32-based smart home that monitors two rooms, raises safety and security alerts, controls appliances through relays, and streams live camera video, all from a single web dashboard on the local network.

## Features

**Room 1 — ESP32 Dev Module**
- Sensors: temperature & humidity (DHT11), gas/smoke (MQ2), flame, motion (PIR), rain, light (LDR)
- Security: NFC card door lock (MFRC522), laser tripwire, ultrasonic distance (HC-SR04)
- Controls: 3 relays (light, fan, outlet) and a buzzer alarm

**Room 2 — ESP32-CAM**
- Live MJPEG camera stream on port 81 and snapshot endpoint
- DHT11, flame, PIR, rain and MQ2 sensors, 3 relays, buzzer, flash LED
- Two firmware versions: `esp32cam_room2` (standard camera module) and `esp32cam_room2_RGB565` (RHYX M21-45 camera, converts RGB565 frames to JPEG)

**Web dashboard**
- Live sensor readings from each room (JSON over HTTP)
- Appliance on/off controls sent as HTTP POST commands
- Alerts for gas, fire, intruders and tripwire breaks
- Embedded live camera view (`camera.html` is a standalone stream viewer)

## Hardware

ESP32 Dev Module · AI Thinker ESP32-CAM · DHT11 · MQ2 · flame sensor · PIR · rain sensor · LDR · HC-SR04 · MFRC522 NFC · laser module · 4-channel relay · 12 V door lock · buzzer

Full pin wiring is documented in the comment block at the top of each `.ino` file.

## Setup

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and the **ESP32** board package.
2. Install libraries: **DHT sensor library** (Adafruit) and **MFRC522**.
3. In each firmware folder, copy `secrets.example.h` to `secrets.h` and enter your WiFi name and password.
4. Upload `firmware/esp32_room1` to the ESP32 and one of the Room 2 sketches to the ESP32-CAM (upload steps are in the file's header comment).
5. Open the Serial Monitor (115200 baud) to see each board's IP address.
6. Open `dashboard/index.html` in a browser on the same WiFi and enter each board's IP address in the dashboard settings (it remembers them in the browser).

## Project structure

```
smartnest-iot/
├── firmware/
│   ├── esp32_room1/
│   ├── esp32cam_room2/
│   └── esp32cam_room2_RGB565/
└── dashboard/
    ├── index.html     # Main control dashboard
    └── camera.html    # Standalone live-stream viewer
```

## License

MIT — see [LICENSE](LICENSE).
