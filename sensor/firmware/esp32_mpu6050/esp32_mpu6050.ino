/*
  ESP32 + MPU-6050  ->  sends motion data to the PC over WiFi (UDP).

  WIRING (MPU-6050 -> ESP32):
    VCC -> 3V3      GND -> GND      SCL -> GPIO 22      SDA -> GPIO 21

  BEFORE UPLOADING:
    1. Type your WiFi name and password below (keep the quote marks).
    2. Arduino IDE: Tools -> Board -> esp32 -> "ESP32 Dev Module", Tools -> Port -> your COM port.
    3. Library needed: "Adafruit MPU6050" (Tools -> Manage Libraries).

  MOUNTING: sensor flat on the car, the X arrow printed on the board pointing FORWARD.
  START-UP: keep the car completely still for 3 seconds after power-on (calibration).

  The PC program (python monitor.py) picks the data up automatically - no IP address needed,
  because the ESP32 broadcasts to every computer on the same WiFi network.
  Each message is one text line:  IMU,<seq>,<millis>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
  (acceleration in m/s^2 with the resting offset removed from ax/ay, rotation in deg/s).
*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ======================= CHANGE THESE =======================
const char* WIFI_NAME     = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
// ============================================================

const uint16_t PC_PORT        = 4210;  // must match IMU_PORT in config.py
const int      SDA_PIN        = 21;
const int      SCL_PIN        = 22;
const int      LED_PIN        = 2;     // on-board blue LED: on = WiFi connected
const int      SAMPLE_HZ      = 100;   // readings sent per second
const int      CALIBRATION_MS = 3000;  // keep the car still this long after power-on

Adafruit_MPU6050 mpu;
WiFiUDP udp;

float biasAx = 0, biasAy = 0, biasGx = 0, biasGy = 0, biasGz = 0;
uint32_t seq = 0;
uint32_t nextSampleUs = 0;
uint32_t lastPrintMs = 0;

const float RAD_TO_DEGF = 57.2957795f;

void connectWifi() {
  Serial.printf("Connecting to WiFi \"%s\"", WIFI_NAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // lower delay
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // blink while connecting
    delay(250);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nStill not connected - check WIFI_NAME / WIFI_PASSWORD (2.4 GHz WiFi only). Retrying...");
      WiFi.disconnect();
      WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
      start = millis();
    }
  }
  digitalWrite(LED_PIN, HIGH);
  Serial.printf("\nWiFi connected. ESP32 IP: %s  - sending to port %u\n",
                WiFi.localIP().toString().c_str(), PC_PORT);
}

void calibrate() {
  Serial.println("Calibrating - keep the car STILL for 3 seconds...");
  double sAx = 0, sAy = 0, sGx = 0, sGy = 0, sGz = 0;
  int n = 0;
  uint32_t start = millis();
  sensors_event_t a, g, temp;
  while (millis() - start < (uint32_t)CALIBRATION_MS) {
    mpu.getEvent(&a, &g, &temp);
    sAx += a.acceleration.x;  sAy += a.acceleration.y;
    sGx += g.gyro.x;          sGy += g.gyro.y;          sGz += g.gyro.z;
    n++;
    delay(5);
  }
  biasAx = sAx / n;  biasAy = sAy / n;
  biasGx = sGx / n;  biasGy = sGy / n;  biasGz = sGz / n;
  Serial.printf("Calibration done (%d readings).\n", n);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  while (!mpu.begin()) {
    Serial.println("MPU-6050 NOT FOUND - check the 4 wires (SDA->21, SCL->22, VCC->3V3, GND->GND)");
    delay(1000);
  }
  Serial.println("MPU-6050 found.");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // built-in smoothing against vibration

  connectWifi();
  calibrate();
  nextSampleUs = micros();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, LOW);
    connectWifi();
  }

  // wait for the next sample time (keeps a steady SAMPLE_HZ)
  if ((int32_t)(micros() - nextSampleUs) < 0) return;
  nextSampleUs += 1000000UL / SAMPLE_HZ;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x - biasAx;
  float ay = a.acceleration.y - biasAy;
  float az = a.acceleration.z;                        // gravity kept (about 9.8 when flat)
  float gx = (g.gyro.x - biasGx) * RAD_TO_DEGF;
  float gy = (g.gyro.y - biasGy) * RAD_TO_DEGF;
  float gz = (g.gyro.z - biasGz) * RAD_TO_DEGF;

  char line[128];
  int len = snprintf(line, sizeof(line), "IMU,%lu,%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                     (unsigned long)seq++, (unsigned long)millis(), ax, ay, az, gx, gy, gz);
  udp.beginPacket(WiFi.broadcastIP(), PC_PORT);
  udp.write((const uint8_t*)line, len);
  udp.endPacket();

  // readable summary in the Serial Monitor (115200 baud), 2 times per second
  if (millis() - lastPrintMs > 500) {
    lastPrintMs = millis();
    Serial.printf("forward %+.2f m/s2   turn %+.1f deg/s   (sent %lu)\n", ax, gz, (unsigned long)seq);
  }
}
