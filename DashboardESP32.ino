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
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="theme-color" content="#1a0638">
<title>Rider Dashboard</title>
<style>
  :root {
    --bg-deep: #12022e;
    --violet: #8b5cf6;
    --magenta: #ec4899;
    --cyan: #22d3ee;
    --lime: #a3e635;
    --emerald: #34d399;
    --orange: #fb923c;
    --amber: #fbbf24;
    --rose: #fb7185;
    --sky: #38bdf8;
    --ink: #f8f5ff;
    --muted: #c4b5fd;
    --card: rgba(30, 10, 66, 0.72);
    --card-line: rgba(196, 181, 253, 0.22);
    --mono: ui-monospace, "SF Mono", "Cascadia Mono", Consolas, monospace;
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    min-height: 100vh;
    color: var(--ink);
    font: 16px/1.5 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
    background:
      radial-gradient(circle at 12% 8%, rgba(236, 72, 153, 0.45), transparent 42%),
      radial-gradient(circle at 92% 18%, rgba(34, 211, 238, 0.35), transparent 40%),
      radial-gradient(circle at 50% 100%, rgba(251, 146, 60, 0.30), transparent 48%),
      linear-gradient(160deg, #2a0a5e 0%, var(--bg-deep) 55%, #0b0120 100%);
    background-attachment: fixed;
  }

  main {
    max-width: 720px;
    margin: 0 auto;
    padding: 20px 16px 40px;
  }

  /* ---------- Header ---------- */

  .hero {
    margin-bottom: 24px;
  }

  .hero-top {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
  }

  h1 {
    margin: 0;
    font-size: 1.75rem;
    line-height: 1.15;
    font-weight: 800;
    letter-spacing: -0.01em;
    background: linear-gradient(90deg, #fde047, var(--magenta) 45%, var(--cyan));
    -webkit-background-clip: text;
    background-clip: text;
    color: transparent;
  }

  .hero-sub {
    margin: 6px 0 0;
    color: var(--muted);
    font-size: 0.95rem;
  }

  .device-pill {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    padding: 6px 14px;
    border-radius: 999px;
    font-size: 0.8rem;
    font-weight: 700;
    color: #0b0120;
    background: linear-gradient(90deg, var(--cyan), var(--lime));
    box-shadow: 0 0 18px rgba(34, 211, 238, 0.55);
    white-space: nowrap;
  }

  .device-pill::before {
    content: "";
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: #0b0120;
    animation: pulse 1.6s ease-in-out infinite;
  }

  /* ---------- Sections ---------- */

  .section {
    margin-top: 28px;
  }

  .section.gps    { --accent: var(--cyan);    --accent-2: var(--violet); }
  .section.maps   { --accent: var(--emerald); --accent-2: var(--cyan); }
  .section.motion { --accent: var(--orange);  --accent-2: var(--magenta); }

  .section-head {
    display: flex;
    flex-wrap: wrap;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    margin-bottom: 12px;
  }

  .section-head h2 {
    margin: 0;
    font-size: 1.2rem;
    font-weight: 800;
    background: linear-gradient(90deg, var(--accent), var(--accent-2));
    -webkit-background-clip: text;
    background-clip: text;
    color: transparent;
  }

  .section-tag {
    padding: 4px 10px;
    border-radius: 999px;
    font-size: 0.75rem;
    font-weight: 700;
    letter-spacing: 0.02em;
    color: var(--accent);
    background: color-mix(in srgb, var(--accent) 16%, transparent);
    border: 1px solid color-mix(in srgb, var(--accent) 55%, transparent);
  }

  /* ---------- Cards ---------- */

  .card {
    position: relative;
    overflow: hidden;
    padding: 18px;
    border-radius: 18px;
    background: var(--card);
    border: 1px solid var(--card-line);
    box-shadow:
      0 10px 30px rgba(0, 0, 0, 0.35),
      0 0 0 1px rgba(255, 255, 255, 0.03) inset;
    -webkit-backdrop-filter: blur(10px);
    backdrop-filter: blur(10px);
  }

  .card::before {
    content: "";
    position: absolute;
    left: 0;
    right: 0;
    top: 0;
    height: 4px;
    background: linear-gradient(90deg, var(--accent), var(--accent-2));
  }

  /* ---------- Profile ---------- */

  .profile {
    display: flex;
    align-items: center;
    gap: 14px;
    margin-bottom: 16px;
  }

  .avatar {
    flex: none;
    display: grid;
    place-items: center;
    width: 56px;
    height: 56px;
    border-radius: 50%;
    color: #fff;
    background: linear-gradient(135deg, var(--magenta), var(--violet) 55%, var(--cyan));
    box-shadow: 0 0 20px rgba(236, 72, 153, 0.55);
  }

  .avatar svg {
    width: 30px;
    height: 30px;
  }

  .profile-name {
    font-weight: 800;
    font-size: 1.1rem;
    color: #fff;
  }

  .profile-detail {
    color: var(--muted);
    font-size: 0.92rem;
  }

  /* ---------- Status banners ---------- */

  .status {
    --tone: var(--sky);
    display: flex;
    align-items: center;
    gap: 10px;
    margin-bottom: 14px;
    padding: 10px 14px;
    border-radius: 12px;
    font-size: 0.92rem;
    font-weight: 600;
    color: var(--tone);
    background: color-mix(in srgb, var(--tone) 14%, transparent);
    border: 1px solid color-mix(in srgb, var(--tone) 50%, transparent);
  }

  .status::before {
    content: "";
    flex: none;
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--tone);
    box-shadow: 0 0 10px var(--tone);
  }

  .status.info  { --tone: var(--sky); }
  .status.ok    { --tone: var(--lime); }
  .status.warn  { --tone: var(--amber); }
  .status.error { --tone: var(--rose); }

  .status.ok::before { animation: pulse 1.6s ease-in-out infinite; }

  /* ---------- Coordinates ---------- */

  .coordinate-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
  }

  .coordinate {
    padding: 14px;
    border-radius: 14px;
    background: linear-gradient(145deg, rgba(34, 211, 238, 0.14), rgba(139, 92, 246, 0.14));
    border: 1px solid rgba(34, 211, 238, 0.35);
  }

  .label {
    display: block;
    font-size: 0.75rem;
    font-weight: 700;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--muted);
  }

  .coordinate-value {
    margin: 4px 0 2px;
    min-height: 1.4em;
    font: 700 1.35rem/1.3 var(--mono);
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
    background: linear-gradient(90deg, var(--cyan), #f0abfc);
    -webkit-background-clip: text;
    background-clip: text;
    color: transparent;
  }

  .coordinate-value.stale {
    background: none;
    color: var(--amber);
  }

  .unit-note {
    font-size: 0.78rem;
    color: rgba(196, 181, 253, 0.8);
  }

  /* ---------- Maps link ---------- */

  .button-link {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 10px;
    min-height: 50px;
    margin-bottom: 16px;
    border-radius: 14px;
    font-weight: 800;
    font-size: 1rem;
    text-decoration: none;
    color: #04261a;
    background: linear-gradient(90deg, var(--lime), var(--emerald) 50%, var(--cyan));
    box-shadow: 0 6px 22px rgba(52, 211, 153, 0.45);
    transition: transform 0.15s ease, box-shadow 0.15s ease, filter 0.15s ease;
  }

  .button-link svg {
    width: 20px;
    height: 20px;
  }

  .button-link:hover {
    transform: translateY(-1px);
    filter: brightness(1.08);
    box-shadow: 0 8px 28px rgba(52, 211, 153, 0.6);
  }

  .button-link:focus-visible,
  .url-field:focus-visible {
    outline: 3px solid #fde047;
    outline-offset: 3px;
  }

  .button-link[aria-disabled="true"] {
    pointer-events: none;
    color: rgba(248, 245, 255, 0.55);
    background: rgba(255, 255, 255, 0.08);
    box-shadow: none;
    border: 1px dashed rgba(196, 181, 253, 0.35);
  }

  .url-field {
    width: 100%;
    margin-top: 6px;
    padding: 12px 14px;
    border-radius: 12px;
    font: 0.85rem/1.4 var(--mono);
    color: var(--emerald);
    background: rgba(4, 38, 26, 0.45);
    border: 1px solid rgba(52, 211, 153, 0.4);
  }

  .url-field::placeholder {
    color: rgba(196, 181, 253, 0.6);
  }

  .helper-text {
    margin: 12px 0 0;
    font-size: 0.85rem;
    color: var(--muted);
  }

  /* ---------- MPU6050 ---------- */

  .sensor-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 12px;
  }

  .sensor-title {
    font-weight: 800;
    font-size: 1.02rem;
    color: #fff;
  }

  .sensor-unit {
    display: inline-block;
    margin: 4px 0 10px;
    padding: 2px 8px;
    border-radius: 6px;
    font-size: 0.75rem;
    font-weight: 700;
    color: var(--orange);
    background: rgba(251, 146, 60, 0.16);
  }

  .axis-row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 0;
    border-top: 1px solid rgba(196, 181, 253, 0.14);
  }

  .axis-row:first-of-type { border-top: 0; }

  .axis-name {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    font-size: 0.9rem;
    color: var(--muted);
  }

  .axis-name::before {
    content: "";
    width: 10px;
    height: 10px;
    border-radius: 3px;
    background: var(--axis);
    box-shadow: 0 0 8px var(--axis);
  }

  .axis-value {
    font: 700 1.1rem/1.2 var(--mono);
    font-variant-numeric: tabular-nums;
    color: var(--axis);
  }

  .axis-row:nth-of-type(1) { --axis: #ff4d8d; }
  .axis-row:nth-of-type(2) { --axis: var(--lime); }
  .axis-row:nth-of-type(3) { --axis: var(--sky); }

  .temperature-row {
    display: flex;
    align-items: baseline;
    gap: 6px;
    margin: 10px 0 4px;
  }

  #temperature {
    font: 800 2.6rem/1 var(--mono);
    font-variant-numeric: tabular-nums;
    background: linear-gradient(90deg, var(--amber), var(--orange) 45%, var(--magenta));
    -webkit-background-clip: text;
    background-clip: text;
    color: transparent;
  }

  .temperature-unit {
    font-size: 1.1rem;
    font-weight: 700;
    color: var(--orange);
  }

  @keyframes pulse {
    0%, 100% { opacity: 1; transform: scale(1); }
    50%      { opacity: 0.45; transform: scale(0.8); }
  }

  @media (max-width: 420px) {
    h1 { font-size: 1.45rem; }
    .coordinate-grid { grid-template-columns: 1fr; }
  }

  @media (prefers-reduced-motion: reduce) {
    * { animation: none !important; transition: none !important; }
  }
</style>
</head>
<body>
<main>
  <header class="hero">
    <div class="hero-top">
      <h1>Rider Dashboard</h1>
      <span class="device-pill">Data Is Sourced By ESP32</span>
    </div>
    <p class="hero-sub">Live GPS position and motion data</p>
  </header>

  <section class="section gps" aria-labelledby="gpsHeading">
    <div class="section-head">
      <h2 id="gpsHeading">Rider Location</h2>
      <span class="section-tag">NEO-6M GPS</span>
    </div>

    <div class="card">
      <div class="profile">
        <div class="avatar" aria-hidden="true">
          <svg viewBox="0 0 24 24" fill="none">
            <circle cx="12" cy="8" r="3.5" stroke="currentColor" stroke-width="1.8"/>
            <path d="M5 20v-1a7 7 0 0 1 14 0v1" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/>
          </svg>
        </div>

        <div class="profile-text">
          <div class="profile-name">Delivery Rider 1</div>
          <div class="profile-detail">Muhammad Irfan Bin Yusof</div>
        </div>
      </div>

      <div id="gpsStatus" class="status info" role="status">Connecting to ESP32…</div>

      <div class="coordinate-grid">
        <div class="coordinate">
          <div class="label">Latitude</div>
          <div id="latitude" class="coordinate-value">—</div>
          <div class="unit-note">Decimal Degrees</div>
        </div>

        <div class="coordinate">
          <div class="label">Longitude</div>
          <div id="longitude" class="coordinate-value">—</div>
          <div class="unit-note">Decimal Degrees</div>
        </div>
      </div>
    </div>
  </section>

  <section class="section maps" aria-labelledby="mapsHeading">
    <div class="section-head">
      <h2 id="mapsHeading">Google Maps</h2>
      <span class="section-tag">External URL Link</span>
    </div>

    <div class="card">
      <div id="mapsStatus" class="status info" role="status">Waiting for fresh GPS coordinates.</div>

      <a id="mapsLink" class="button-link" target="_blank" rel="noopener noreferrer" aria-disabled="true" tabindex="-1">
        <svg viewBox="0 0 24 24" fill="none" aria-hidden="true">
          <path d="M14 3h7v7M21 3 10 14M10 3H5a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-5"
                stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>
        </svg>
        Open in Google Maps
      </a>

      <label class="label" for="mapsURL">Location URL</label>
      <input id="mapsURL" class="url-field" type="text" readonly
             placeholder="Available when GPS coordinates are fresh"
             aria-label="Google Maps URL from NEO-6M coordinates">

      <p class="helper-text">
        Opens the NEO-6M's reported coordinates separately.
        Your phone requires Internet access to load Google Maps.
      </p>
    </div>
  </section>

  <section class="section motion" aria-labelledby="mpuHeading">
    <div class="section-head">
      <h2 id="mpuHeading">Motion Sensor</h2>
      <span class="section-tag">MPU6050 Accelerometer</span>
    </div>

    <div id="mpuStatus" class="status info" role="status">Waiting for MPU6050 data…</div>

    <div class="sensor-grid">
      <div class="card">
        <div class="sensor-title">Acceleration</div>
        <div class="sensor-unit">m/s²</div>
        <div class="axis-row"><span class="axis-name">X axis</span><span id="ax" class="axis-value">—</span></div>
        <div class="axis-row"><span class="axis-name">Y axis</span><span id="ay" class="axis-value">—</span></div>
        <div class="axis-row"><span class="axis-name">Z axis</span><span id="az" class="axis-value">—</span></div>
      </div>

      <div class="card">
        <div class="sensor-title">Angular velocity</div>
        <div class="sensor-unit">rad/s</div>
        <div class="axis-row"><span class="axis-name">X axis</span><span id="gx" class="axis-value">—</span></div>
        <div class="axis-row"><span class="axis-name">Y axis</span><span id="gy" class="axis-value">—</span></div>
        <div class="axis-row"><span class="axis-name">Z axis</span><span id="gz" class="axis-value">—</span></div>
      </div>

      <div class="card">
        <div class="sensor-title">Sensor temperature</div>
        <div class="temperature-row">
          <span id="temperature">—</span>
          <span class="temperature-unit">°C</span>
        </div>
        <div class="unit-note">MPU6050 chip temperature</div>
      </div>
    </div>
  </section>
</main>

<script>
  const POLL_MS = 1000;
  const $ = (id) => document.getElementById(id);

  function setStatus(id, tone, text) {
    const el = $(id);
    el.className = 'status ' + tone;
    el.textContent = text;
  }

  function fmt(value, digits) {
    return (typeof value === 'number' && isFinite(value)) ? value.toFixed(digits) : '—';
  }

  function updateGPS(d) {
    const lat = $('latitude');
    const lng = $('longitude');

    lat.textContent = d.valid ? fmt(d.latitude, 6) : '—';
    lng.textContent = d.valid ? fmt(d.longitude, 6) : '—';
    lat.classList.toggle('stale', d.valid && !d.fresh);
    lng.classList.toggle('stale', d.valid && !d.fresh);

    if (!d.receiving) {
      setStatus('gpsStatus', 'error', 'No data from the GPS module. Check wiring (GPS TX → GPIO16).');
    } else if (!d.valid) {
      setStatus('gpsStatus', 'warn', 'GPS connected, waiting for a satellite fix…');
    } else if (!d.fresh) {
      setStatus('gpsStatus', 'warn', 'GPS fix is stale (last update ' + Math.round(d.age_ms / 1000) + ' s ago).');
    } else {
      setStatus('gpsStatus', 'ok', 'Live GPS fix');
    }

    const link = $('mapsLink');
    const field = $('mapsURL');

    if (d.fresh) {
      const url = 'https://www.google.com/maps/search/?api=1&query=' +
                  d.latitude.toFixed(6) + ',' + d.longitude.toFixed(6);
      link.href = url;
      link.setAttribute('aria-disabled', 'false');
      link.removeAttribute('tabindex');
      field.value = url;
      setStatus('mapsStatus', 'ok', 'Location link ready.');
    } else {
      link.removeAttribute('href');
      link.setAttribute('aria-disabled', 'true');
      link.setAttribute('tabindex', '-1');
      field.value = '';
      setStatus('mapsStatus', 'info', 'Waiting for fresh GPS coordinates.');
    }
  }

  function updateMPU(m) {
    const ids = ['ax', 'ay', 'az', 'gx', 'gy', 'gz'];

    if (!m || !m.detected) {
      setStatus('mpuStatus', 'error', 'MPU6050 not detected. Check SDA → GPIO21 and SCL → GPIO22.');
    } else if (!m.valid) {
      setStatus('mpuStatus', 'warn', 'MPU6050 detected, but the last reading failed.');
    } else {
      setStatus('mpuStatus', 'ok', 'Live MPU6050 data');
    }

    const valid = m && m.valid;
    ids.forEach((id) => { $(id).textContent = valid ? fmt(m[id], 2) : '—'; });
    $('temperature').textContent = valid ? fmt(m.temperature, 1) : '—';
  }

  function showOffline() {
    setStatus('gpsStatus', 'error', 'Cannot reach the ESP32. Stay connected to the ESP32-GPS Wi-Fi.');
    setStatus('mpuStatus', 'error', 'Cannot reach the ESP32.');
  }

  async function poll() {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), 3000);

    try {
      const response = await fetch('/api/gps', { cache: 'no-store', signal: controller.signal });
      if (!response.ok) throw new Error('HTTP ' + response.status);
      const data = await response.json();
      updateGPS(data);
      updateMPU(data.mpu);
    } catch (error) {
      showOffline();
    } finally {
      clearTimeout(timer);
      setTimeout(poll, POLL_MS);
    }
  }

  poll();
</script>
</body>
</html>
)HTML";

// ---------- GPS input ----------

void readGPS() {
  while (gpsSerial.available() > 0) {
    const int value = gpsSerial.read();

    if (value < 0) {
      break;
    }

    gps.encode(static_cast<char>(value));

    lastGPSByteTime = millis();
    receivedGPSBytes = true;
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

    mpuValid =
      eventOK &&
      isfinite(acceleration.acceleration.x) &&
      isfinite(acceleration.acceleration.y) &&
      isfinite(acceleration.acceleration.z) &&
      isfinite(angularVelocity.gyro.x) &&
      isfinite(angularVelocity.gyro.y) &&
      isfinite(angularVelocity.gyro.z) &&
      isfinite(temperature.temperature);
  }

  // Drain pending GPS bytes before taking one snapshot.
  readGPS();

  const uint32_t now = millis();

  const bool receiving =
    receivedGPSBytes &&
    static_cast<uint32_t>(now - lastGPSByteTime) < GPS_STREAM_TIMEOUT_MS;

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

  json += "{\"valid\":";
  json += locationValid ? "true" : "false";

  json += ",\"fresh\":";
  json += fresh ? "true" : "false";

  json += ",\"receiving\":";
  json += receiving ? "true" : "false";

  json += ",\"latitude\":";
  if (locationValid) {
    json += String(latitude, 6);
  } else {
    json += "null";
  }

  json += ",\"longitude\":";
  if (locationValid) {
    json += String(longitude, 6);
  } else {
    json += "null";
  }

  json += ",\"age_ms\":";
  if (locationValid) {
    json += String(age);
  } else {
    json += "null";
  }

  json += ",\"satellites\":";
  if (satellitesValid) {
    json += String(satellites);
  } else {
    json += "null";
  }

  json += ",\"characters\":";
  json += String(characters);

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

  json += ",\"gx\":";
  appendNumber(json, angularVelocity.gyro.x, mpuValid);

  json += ",\"gy\":";
  appendNumber(json, angularVelocity.gyro.y, mpuValid);

  json += ",\"gz\":";
  appendNumber(json, angularVelocity.gyro.z, mpuValid);

  json += ",\"temperature\":";
  appendNumber(json, temperature.temperature, mpuValid);

  json += "}";

  json += ",\"router_connected\":";
  json += routerConnected ? "true" : "false";

  json += ",\"router_ip\":";
  if (routerConnected) {
    json += "\"";
    json += routerIP;
    json += "\"";
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

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");

    server.send_P(
      200,
      "text/html; charset=utf-8",
      INDEX_HTML
    );
  });

  server.on("/api/gps", HTTP_GET, sendGPSData);

  server.onNotFound([]() {
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
