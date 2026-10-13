#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// Wifi profile need update

const char *AP_SSID = "ESP32-GPS";
const char *AP_PASSWORD = "GPSdashboard123";

const char *ROUTER_SSID = "";
const char *ROUTER_PASSWORD = "";

IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

// GPS SETTING (FIX NI SEBAB CALIBRATION LARI)

constexpr int GPS_RX_PIN = 16;
constexpr int GPS_TX_PIN = 17;
constexpr uint32_t GPS_BAUD = 9600;
constexpr uint32_t GPS_STREAM_TIMEOUT_MS = 5000;

//MPU6050 configuration

constexpr int MPU_SDA_PIN = 21;
constexpr int MPU_SCL_PIN = 22;

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
WebServer server(80);

uint32_t lastGPSByteTime = 0;
bool receivedGPSBytes = false;
bool mpuDetected = false;

// Embedded HTML / CSS / JS

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Live Rider Dashboard</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<style>
  * { box-sizing: border-box; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background: #f4f4f9;
    color: #333;
    margin: 0;
    padding: 16px;
  }
  .container {
    max-width: 800px;
    margin: 0 auto;
  }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-bottom: 16px;
  }
  h1 { font-size: 1.3rem; margin: 0; }
  .card {
    background: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 12px;
    padding: 16px;
    margin-bottom: 16px;
    box-shadow: 0 1px 3px rgba(0,0,0,0.05);
  }
  .status {
    padding: 8px 12px;
    border-radius: 6px;
    font-size: 0.85rem;
    font-weight: 500;
    margin-bottom: 16px;
  }
  .status.good { background: #dcfce7; color: #166534; }
  .status.bad { background: #fee2e2; color: #991b1b; }
  #map {
    height: 320px;
    border-radius: 8px;
    border: 1px solid #cbd5e1;
    margin-top: 8px;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
    gap: 12px;
  }
  .stat-box {
    background: #f8fafc;
    border: 1px solid #e2e8f0;
    padding: 12px;
    border-radius: 8px;
    text-align: center;
  }
  .stat-value {
    font-size: 1.25rem;
    font-weight: 700;
    color: #2563eb;
    font-variant-numeric: tabular-nums;
  }
  .stat-label {
    font-size: 0.75rem;
    color: #64748b;
    margin-top: 4px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
  }
  .coord-text {
    font-size: 0.85rem;
    color: #475569;
    margin-top: 8px;
  }
</style>
</head>
<body>

<div class="container">
  <header>
    <h1>Rider Dashboard</h1>
    <span id="connectionStatus" class="status bad">Connecting...</span>
  </header>

  <div class="card">
    <h3 style="margin-top:0;">Live GPS Map</h3>
    <div id="map"></div>
    <div class="coord-text">
      Latitude: <span id="lat">--</span> | Longitude: <span id="lng">--</span>
    </div>
  </div>

  <div class="card">
    <h3 style="margin-top:0;">MPU6050 Accelerometer & Temperature</h3>
    <div class="grid">
      <div class="stat-box"><div id="ax" class="stat-value">--</div><div class="stat-label">Accel X</div></div>
      <div class="stat-box"><div id="ay" class="stat-value">--</div><div class="stat-label">Accel Y</div></div>
      <div class="stat-box"><div id="az" class="stat-value">--</div><div class="stat-label">Accel Z</div></div>
      <div class="stat-box"><div id="temp" class="stat-value">--</div><div class="stat-label">Temp (°C)</div></div>
    </div>
  </div>
</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
  let map = L.map('map').setView([0, 0], 2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap'
  }).addTo(map);

  let marker = null;
  let mapInitialized = false;

  function updateUI(data) {
    const conn = document.getElementById('connectionStatus');
    conn.textContent = "Connected to ESP32";
    conn.className = "status good";

    if (data.valid && data.latitude !== null && data.longitude !== null) {
      document.getElementById('lat').textContent = data.latitude.toFixed(6);
      document.getElementById('lng').textContent = data.longitude.toFixed(6);

      const latLng = [data.latitude, data.longitude];
      if (!mapInitialized) {
        map.setView(latLng, 16);
        marker = L.marker(latLng).addTo(map);
        mapInitialized = true;
      } else {
        marker.setLatLng(latLng);
        map.panTo(latLng);
      }
    }

    if (data.mpu && data.mpu.valid) {
      document.getElementById('ax').textContent = data.mpu.ax.toFixed(2);
      document.getElementById('ay').textContent = data.mpu.ay.toFixed(2);
      document.getElementById('az').textContent = data.mpu.az.toFixed(2);
      document.getElementById('temp').textContent = data.mpu.temperature.toFixed(2);
    }
  }

  async function poll() {
    try {
      const response = await fetch('/api/gps', { cache: 'no-store' });
      if (!response.ok) throw new Error();
      const data = await response.json();
      updateUI(data);
    } catch (e) {
      const conn = document.getElementById('connectionStatus');
      conn.textContent = "ESP32 Unreachable";
      conn.className = "status bad";
    } finally {
      setTimeout(poll, 1000);
    }
  }

  poll();
</script>
</body>
</html>
)HTML";

// GPS READER NEED TO BE RECALIBRATE

void readGPS() {
  while (gpsSerial.available() > 0) {
    int value = gpsSerial.read();
    if (value < 0) break;
    gps.encode(static_cast<char>(value));
    lastGPSByteTime = millis();
    receivedGPSBytes = true;
  }
}

void appendNumber(String &json, float value, bool valid) {
  if (valid && isfinite(value)) {
    json += String(value, 3);
  } else {
    json += "null";
  }
}

// API

void sendGPSData() {
  readGPS();

  sensors_event_t acceleration = {};
  sensors_event_t angularVelocity = {};
  sensors_event_t temperature = {};
  bool mpuValid = false;

  if (mpuDetected) {
    bool eventOK = mpu.getEvent(&acceleration, &angularVelocity, &temperature);
    mpuValid = eventOK && isfinite(acceleration.acceleration.x) && isfinite(temperature.temperature);
  }

  readGPS();

  bool rawLocationValid = gps.location.isValid();
  double latitude = rawLocationValid ? gps.location.lat() : 0.0;
  double longitude = rawLocationValid ? gps.location.lng() : 0.0;

  String json;
  json.reserve(512);

  json += "{\"valid\":";
  json += rawLocationValid ? "true" : "false";
  json += ",\"latitude\":";
  if (rawLocationValid) json += String(latitude, 6); else json += "null";
  json += ",\"longitude\":";
  if (rawLocationValid) json += String(longitude, 6); else json += "null";

  json += ",\"mpu\":{\"detected\":";
  json += mpuDetected ? "true" : "false";
  json += ",\"valid\":";
  json += mpuValid ? "true" : "false";
  json += ",\"ax\":";
  appendNumber(json, acceleration.acceleration.x, mpuValid);
  json += ",\"ay\":";
  appendNumber(json, acceleration.acceleration.y, mpuValid);
  json += ",\"az\":";
  appendNumber(json, acceleration.acceleration.z, mpuValid);
  json += ",\"temperature\":";
  appendNumber(json, temperature.temperature, mpuValid);
  json += "}}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// SETUP FOR IOT HARDWARE

void setup() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  mpuDetected = mpu.begin(0x68, &Wire);

  if (mpuDetected) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
 //                                                     __                          
// ___.__. ____   ______    ____   ____   _____   ____ |  | __ _____ _____    ____  
//<   |  |/ __ \ /  ___/   / ___\_/ __ \ /     \ /  _ \|  |/ //     \\__  \  /    \ 
// \___  \  ___/ \___ \   / /_/  >  ___/|  Y Y  (  <_> )    <|  Y Y  \/ __ \|   |  \
// / ____|\___  >____  >  \___  / \___  >__|_|  /\____/|__|_ \__|_|  (____  /___|  /
// \/         \/     \/  /_____/      \/      \/            \/     \/     \/     \/ 
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });

  server.on("/api/gps", HTTP_GET, sendGPSData);
  server.begin();
}
// MAIN LOOPER
void loop() {
  readGPS();
  server.handleClient();
  delay(1);
}
