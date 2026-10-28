Act as an expert embedded systems and web developer. I have the following Arduino ESP32 web server code that hosts a live rider's dashboard with GPS and MPU6050 sensor data.

I have a Google Maps Embed API key. I want you to modify the HTML/JavaScript inside the `INDEX_HTML` string of this code to replace or enhance the current external Google Maps link card with a live embedded Google Map (using the Google Maps Embed API) that dynamically updates or centers based on the real-time latitude and longitude coordinates received from the ESP32 GPS module.

Please provide the complete, updated ESP32 code with the embedded map integrated cleanly into the existing purple-themed dashboard layout. Use a placeholder string like "YOUR_GOOGLE_MAPS_EMBED_API_KEY" for the API key so I can insert mine easily.

Here is my current code:

#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ---------- Wi-Fi configuration ----------

const char *AP_SSID = "ESP32-GPS";
const char *AP_PASSWORD = "GPSdashboard123";

// Optional Internet-connected router.
// Leave empty for AP-only operation.
const char *ROUTER_SSID = "";
const char *ROUTER_PASSWORD = "";

IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

// ---------- GPS configuration ----------

constexpr int GPS_RX_PIN = 16;
constexpr int GPS_TX_PIN = 17;
constexpr uint32_t GPS_BAUD = 9600;
constexpr uint32_t GPS_FRESHNESS_MS = 5000;
constexpr uint32_t GPS_STREAM_TIMEOUT_MS = 5000;

// ---------- MPU6050 configuration ----------

constexpr int MPU_SDA_PIN = 21;
constexpr int MPU_SCL_PIN = 22;

// UART1 explicitly mapped to GPIO16/GPIO17.
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
WebServer server(80);

uint32_t lastGPSByteTime = 0;
bool receivedGPSBytes = false;
bool mpuDetected = false;

// ---------- Embedded website ----------

const char INDEX_HTML[] PROGMEM = R"HTML(

```
<span class="device-pill">Data Is Sourced By ESP32</span>

```

```
<div class="card">
  <div class="profile">
    <div class="avatar" aria-hidden="true">
      <svg viewBox="0 0 24 24" fill="none">
        <circle
          cx="12"
          cy="8"
          r="3.5"
          stroke="currentColor"
          stroke-width="1.8"
        />
        <path
          d="M5 20v-1a7 7 0 0 1 14 0v1"
          stroke="currentColor"
          stroke-width="1.8"
          stroke-linecap="round"
        />
      </svg>
    </div>

    <div class="profile-text">
      <div class="profile-name">Delivery Rider 1</div>
      <div class="profile-detail">
        Muhammad Irfan Bin Yusof
      </div>
    </div>
  </div>

  <div id="gpsStatus" class="status info" role="status">
    Connecting to ESP32…
  </div>

  <div class="coordinate-grid">
    <div class="coordinate">
      <div class="label">Latitude</div>
      <div id="latitude" class="coordinate-value"></div>
      <div class="unit-note">Decimal Degrees</div>
    </div>

    <div class="coordinate">
      <div class="label">Longitude</div>
      <div id="longitude" class="coordinate-value"></div>
      <div class="unit-note">Decimal Degrees</div>
    </div>
  </div>
</div>

```

```
  <span class="section-tag">External URL Link</span>
</div>

<div class="card">
  <div id="mapsStatus" class="status info" role="status">
    Waiting for fresh GPS coordinates.
  </div>

  <a
    id="mapsLink"
    class="button-link"
    target="_blank"
    rel="noopener noreferrer"
    aria-disabled="true"
    tabindex="-1"
  >
    <svg viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M14 3h7v7M21 3 10 14M10 3H5a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-5"
        stroke="currentColor"
        stroke-width="1.8"
        stroke-linecap="round"
        stroke-linejoin="round"
      />
    </svg>
    Open in Google Maps
  </a>

  <label class="label" for="mapsURL">Location URL</label>

  <input
    id="mapsURL"
    class="url-field"
    type="text"
    readonly
    placeholder="Available when GPS coordinates are fresh"
    aria-label="Google Maps URL from NEO-6M coordinates"
  >

  <p class="helper-text">
    Opens the NEO-6M's reported coordinates separately.
    Your phone requires Internet access to load Google Maps.
  </p>
</div>

```

```
  <span class="section-tag">MPU6050 Accelerometer</span>
</div>

<div id="mpuStatus" class="status info" role="status">
  Waiting for MPU6050 data…
</div>

<div class="sensor-grid">
  <div class="card">
    <div class="sensor-title">Acceleration</div>
    <div class="sensor-unit">m/s²</div>

    <div class="axis-row">
      <span class="axis-name">X axis</span>
      <span id="ax" class="axis-value"></span>
    </div>

    <div class="axis-row">
      <span class="axis-name">Y axis</span>
      <span id="ay" class="axis-value"></span>
    </div>

    <div class="axis-row">
      <span class="axis-name">Z axis</span>
      <span id="az" class="axis-value"></span>
    </div>
  </div>

  <div class="card">
    <div class="sensor-title">Angular velocity</div>
    <div class="sensor-unit">rad/s</div>

    <div class="axis-row">
      <span class="axis-name">X axis</span>
      <span id="gx" class="axis-value"></span>
    </div>

    <div class="axis-row">
      <span class="axis-name">Y axis</span>
      <span id="gy" class="axis-value"></span>
    </div>

    <div class="axis-row">
      <span class="axis-name">Z axis</span>
      <span id="gz" class="axis-value"></span>
    </div>
  </div>

  <div class="card">
    <div class="sensor-title">Sensor temperature</div>

    <div class="temperature-row">
      <span id="temperature"></span>
      <span class="temperature-unit">°C</span>
    </div>

    <div class="unit-note">MPU6050 chip temperature</div>
  </div>
</div>

```

// ---------- GPS input ----------

void readGPS() {
while (gpsSerial.available() > 0) {
const int value = gpsSerial.read();

```
if (value < 0) {
  break;
}

gps.encode(static_cast<char>(value));

lastGPSByteTime = millis();
receivedGPSBytes = true;

```

}
}

// ---------- JSON number helper ----------

void appendNumber(String &json, float value, bool valid) {
if (valid && isfinite(value)) {
json += String(value, 3);
} else {
json += "null";
}
}

// ---------- Combined GPS + MPU6050 API ----------

void sendGPSData() {
readGPS();

sensors_event_t acceleration = {};
sensors_event_t angularVelocity = {};
sensors_event_t temperature = {};

bool mpuValid = false;

if (mpuDetected) {
const bool eventOK = mpu.getEvent(
&acceleration,
&angularVelocity,
&temperature
);

```
mpuValid =
  eventOK &&
  isfinite(acceleration.acceleration.x) &&
  isfinite(acceleration.acceleration.y) &&
  isfinite(acceleration.acceleration.z) &&
  isfinite(angularVelocity.gyro.x) &&
  isfinite(angularVelocity.gyro.y) &&
  isfinite(angularVelocity.gyro.z) &&
  isfinite(temperature.temperature);

```

}

// Drain pending GPS bytes before taking one snapshot.
readGPS();

const uint32_t now = millis();

const bool receiving =
receivedGPSBytes &&
static_cast<uint32_t>(now - lastGPSByteTime) <
GPS_STREAM_TIMEOUT_MS;

const bool rawLocationValid = gps.location.isValid();

const double latitude =
rawLocationValid ? gps.location.lat() : 0.0;

const double longitude =
rawLocationValid ? gps.location.lng() : 0.0;

const unsigned long age = gps.location.age();

const bool locationValid =
rawLocationValid &&
isfinite(latitude) &&
isfinite(longitude) &&
latitude >= -90.0 &&
latitude <= 90.0 &&
longitude >= -180.0 &&
longitude <= 180.0;

const bool fresh =
locationValid &&
age < GPS_FRESHNESS_MS;

// Retained in JSON for compatibility; not displayed.
const bool satellitesValid = gps.satellites.isValid();

const uint32_t satellites =
satellitesValid ? gps.satellites.value() : 0;

const unsigned long characters = gps.charsProcessed();

const bool routerConnected =
WiFi.status() == WL_CONNECTED;

const String routerIP =
routerConnected ? WiFi.localIP().toString() : "";

String json;
json.reserve(850);

json += "{"valid":";
json += locationValid ? "true" : "false";

json += ","fresh":";
json += fresh ? "true" : "false";

json += ","receiving":";
json += receiving ? "true" : "false";

json += ","latitude":";
if (locationValid) {
json += String(latitude, 6);
} else {
json += "null";
}

json += ","longitude":";
if (locationValid) {
json += String(longitude, 6);
} else {
json += "null";
}

json += ","age_ms":";
if (locationValid) {
json += String(age);
} else {
json += "null";
}

json += ","satellites":";
if (satellitesValid) {
json += String(satellites);
} else {
json += "null";
}

json += ","characters":";
json += String(characters);

json += ","mpu":{"detected":";
json += mpuDetected ? "true" : "false";

json += ","valid":";
json += mpuValid ? "true" : "false";

json += ","ax":";
appendNumber(json, acceleration.acceleration.x, mpuValid);

json += ","ay":";
appendNumber(json, acceleration.acceleration.y, mpuValid);

json += ","az":";
appendNumber(json, acceleration.acceleration.z, mpuValid);

json += ","gx":";
appendNumber(json, angularVelocity.gyro.x, mpuValid);

json += ","gy":";
appendNumber(json, angularVelocity.gyro.y, mpuValid);

json += ","gz":";
appendNumber(json, angularVelocity.gyro.z, mpuValid);

json += ","temperature":";
appendNumber(json, temperature.temperature, mpuValid);

json += "}";

json += ","router_connected":";
json += routerConnected ? "true" : "false";

json += ","router_ip":";
if (routerConnected) {
json += """;
json += routerIP;
json += """;
} else {
json += "null";
}

json += "}";

server.sendHeader("Cache-Control", "no-store");
server.send(200, "application/json", json);
}

// ---------- Setup ----------

void setup() {
gpsSerial.setRxBufferSize(2048);

gpsSerial.begin(
GPS_BAUD,
SERIAL_8N1,
GPS_RX_PIN,
GPS_TX_PIN
);

Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
Wire.setTimeOut(50);

mpuDetected = mpu.begin(0x68, &Wire);

if (mpuDetected) {
mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
mpu.setGyroRange(MPU6050_RANGE_500_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

const bool routerConfigured =
ROUTER_SSID != nullptr &&
ROUTER_SSID[0] != '\0';

WiFi.mode(routerConfigured ? WIFI_AP_STA : WIFI_AP);

WiFi.softAPConfig(apIP, apGateway, apSubnet);
WiFi.softAP(AP_SSID, AP_PASSWORD);

if (routerConfigured) {
WiFi.setAutoReconnect(true);
WiFi.begin(ROUTER_SSID, ROUTER_PASSWORD);
}

server.on("/", HTTP_GET, [](https://www.google.com/search?q=) {
server.sendHeader("Cache-Control", "no-store");

```
server.send_P(
  200,
  "text/html; charset=utf-8",
  INDEX_HTML
);

```

});

server.on("/api/gps", HTTP_GET, sendGPSData);

server.onNotFound([](https://www.google.com/search?q=) {
server.sendHeader("Cache-Control", "no-store");
server.send(404, "text/plain", "Not found");
});

server.begin();
}

// ---------- Main loop ----------

void loop() {
readGPS();
server.handleClient();
readGPS();
delay(1);
}