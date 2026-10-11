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
<meta name="theme-color" content="#f5f2fb">
<title>Live Rider's Dashboard</title>

<!-- External font dependency: requires browser Internet access. -->
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link
  rel="stylesheet"
  href="https://fonts.googleapis.com/css2?family=Roboto:wght@400;500;600;700;800&display=swap"
>

<style>
:root {
  color-scheme: light;

  --background: #f5f2fb;
  --surface: #ffffff;
  --soft: #f8f6fc;

  --purple: #7252c7;
  --purple-dark: #5738a4;
  --purple-light: #eee7fb;

  --text: #292337;
  --muted: #746b83;
  --border: #e9e3f0;

  --shadow: 0 8px 28px rgba(65, 42, 100, .06);
  --font: "Roboto", Arial, sans-serif;
}

* {
  box-sizing: border-box;
}

html {
  font-family: var(--font);
}

body {
  margin: 0;
  min-height: 100vh;

  color: var(--text);

  background:
    radial-gradient(
      ellipse at 90% 0%,
      #ede5fb 0,
      transparent 45%
    ),
    var(--background);

  font-family: var(--font);
  font-size: 15px;
  line-height: 1.5;
  -webkit-font-smoothing: antialiased;
}

button,
input,
select,
textarea,
a {
  font-family: var(--font);
}

button,
input,
a {
  -webkit-tap-highlight-color: transparent;
}

main {
  width: 100%;
  max-width: 860px;
  margin: 0 auto;
  padding: 26px 18px 30px;
}

.topbar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  margin-bottom: 24px;
}

.brand-name {
  font-size: 1.05rem;
  font-weight: 800;
  letter-spacing: -.025em;
}

.brand-subtitle {
  margin-top: 2px;
  color: var(--muted);
  font-size: .72rem;
  font-weight: 400;
}

.device-pill {
  flex-shrink: 0;
  color: var(--purple-dark);
  background: var(--purple-light);
  border: 1px solid #e1d6f6;
  border-radius: 999px;
  padding: 7px 11px;
  font-size: .72rem;
  font-weight: 700;
}

h1 {
  margin: 0 0 22px;
  font-size: clamp(1.55rem, 4vw, 2rem);
  line-height: 1.25;
  font-weight: 800;
  letter-spacing: -.035em;
}

.section-heading {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  margin: 25px 2px 12px;
}

h2 {
  margin: 0;
  font-size: 1rem;
  font-weight: 700;
  letter-spacing: -.015em;
}

.section-tag {
  color: var(--muted);
  font-size: .72rem;
  font-weight: 500;
}

.card {
  padding: 22px;
  background: var(--surface);
  border: 1px solid var(--border);
  border-radius: 24px;
  box-shadow: var(--shadow);
}

.profile {
  display: flex;
  align-items: center;
  gap: 13px;
  margin-bottom: 19px;
}

.avatar {
  display: grid;
  place-items: center;
  width: 58px;
  height: 58px;
  flex: 0 0 58px;
  border-radius: 50%;
  color: var(--purple);
  background: var(--purple-light);
  border: 4px solid #f8f5ff;
}

.avatar svg {
  width: 30px;
  height: 30px;
}

.profile-text {
  min-width: 0;
}

.profile-name {
  font-size: 1.12rem;
  font-weight: 700;
  letter-spacing: -.015em;
}

.profile-detail {
  margin-top: 3px;
  font-size: .78rem;
  color: var(--muted);
  overflow-wrap: anywhere;
}

.status {
  display: flex;
  align-items: flex-start;
  gap: 9px;
  border: 1px solid;
  border-radius: 14px;
  padding: 12px 13px;
  font-size: .8rem;
  line-height: 1.5;
  margin-bottom: 17px;
}

.status::before {
  content: "";
  flex: 0 0 7px;
  width: 7px;
  height: 7px;
  border-radius: 50%;
  margin-top: 5px;
  background: currentColor;
}

.info {
  color: #61479a;
  background: #f3eefc;
  border-color: #e7dcf7;
}

.good {
  color: #23664e;
  background: #edf8f1;
  border-color: #d4ebdd;
}

.warn {
  color: #805814;
  background: #fff8e9;
  border-color: #f2e2bb;
}

.bad {
  color: #a13f50;
  background: #fff0f3;
  border-color: #f3d5dd;
}

.coordinate-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
}

.coordinate {
  min-width: 0;
  padding: 16px;
  background: var(--soft);
  border: 1px solid #eee8f4;
  border-radius: 17px;
}

.label {
  display: block;
  color: var(--muted);
  font-size: .73rem;
  font-weight: 600;
  margin-bottom: 7px;
}

.coordinate-value {
  min-height: 1.8rem;
  color: var(--purple-dark);
  font-size: clamp(1.3rem, 3.8vw, 1.75rem);
  font-weight: 700;
  line-height: 1.3;
  letter-spacing: -.025em;
  font-variant-numeric: tabular-nums;
  overflow-wrap: anywhere;
}

.unit-note {
  color: var(--muted);
  font-size: .69rem;
  margin-top: 5px;
}

.heading-with-icon {
  display: flex;
  align-items: center;
  gap: 9px;
}

.section-icon {
  display: grid;
  place-items: center;
  flex: 0 0 32px;
  width: 32px;
  height: 32px;
  border-radius: 10px;
  color: var(--purple);
  background: var(--purple-light);
}

.section-icon svg {
  width: 22px;
  height: 22px;
}

.button-link {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 9px;
  width: 100%;
  min-height: 49px;
  padding: 13px 18px;
  margin-bottom: 19px;

  color: #ffffff;
  background: var(--purple);
  border: 1px solid var(--purple);
  border-radius: 999px;

  font-family: var(--font);
  font-size: .9rem;
  font-weight: 700;
  text-decoration: none;
  cursor: pointer;

  transition:
    background .15s ease,
    opacity .15s ease;
}

.button-link svg {
  width: 18px;
  height: 18px;
}

.button-link:hover:not([aria-disabled="true"]) {
  background: var(--purple-dark);
  border-color: var(--purple-dark);
}

.button-link[aria-disabled="true"] {
  opacity: .4;
  cursor: not-allowed;
}

a:focus-visible,
input:focus-visible {
  outline: 3px solid #a78bdf;
  outline-offset: 3px;
}

.url-field {
  width: 100%;
  padding: 12px 13px;
  border: 1px solid var(--border);
  border-radius: 13px;
  background: var(--soft);
  color: #51465f;
  font-family: var(--font);
  font-size: .78rem;
}

.url-field::placeholder {
  color: var(--muted);
}

.helper-text {
  margin: 11px 0 0;
  color: var(--muted);
  font-size: .75rem;
  line-height: 1.6;
}

.sensor-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 14px;
}

.sensor-title {
  font-size: .87rem;
  font-weight: 700;
  margin-bottom: 3px;
}

.sensor-unit {
  color: var(--purple);
  font-size: .73rem;
  font-weight: 500;
  margin-bottom: 13px;
}

.axis-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  min-height: 42px;
  border-bottom: 1px solid #f0ebf5;
}

.axis-row:last-child {
  border-bottom: 0;
}

.axis-name {
  color: var(--muted);
  font-size: .78rem;
}

.axis-value {
  min-height: 1.4rem;
  color: #40324f;
  font-size: 1.08rem;
  font-weight: 700;
  font-variant-numeric: tabular-nums;
}

.temperature-row {
  display: flex;
  align-items: baseline;
  gap: 6px;
  margin: 12px 0 4px;
}

#temperature {
  min-height: 2.3rem;
  color: var(--purple-dark);
  font-size: 2rem;
  font-weight: 700;
  letter-spacing: -.025em;
  font-variant-numeric: tabular-nums;
}

.temperature-unit {
  color: var(--muted);
  font-size: 1rem;
}

footer {
  margin-top: 24px;
  text-align: center;
  color: var(--muted);
  font-size: .72rem;
}

#lastResponse {
  color: #514260;
  font-weight: 600;
  font-variant-numeric: tabular-nums;
}

@media (max-width: 479px) {
  main {
    padding: 20px 14px 26px;
  }

  .card {
    padding: 18px;
    border-radius: 21px;
  }

  .coordinate-grid,
  .sensor-grid {
    grid-template-columns: 1fr;
  }

  .topbar {
    margin-bottom: 21px;
  }

  .device-pill {
    font-size: .66rem;
    padding: 6px 9px;
  }

  .section-tag {
    font-size: .65rem;
  }
}
</style>
</head>

<body>
<main>
  <div class="topbar">
    <div class="brand">
      <div class="brand-name">Rider Monitor</div>
    </div>

    <span class="device-pill">Data Is Sourced By ESP32</span>
  </div>

  <h1>Realtime Delivery Rider's Dashboard</h1>

  <section aria-labelledby="gpsHeading">
    <div class="section-heading">
      <h2 id="gpsHeading">Rider's Location</h2>
      <span class="section-tag">NEO-6M GPS Module</span>
    </div>

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
  </section>

  <section aria-labelledby="mapsHeading">
    <div class="section-heading">
      <h2 id="mapsHeading" class="heading-with-icon">
        <span class="section-icon" aria-hidden="true">
          <svg viewBox="0 0 24 24" fill="none">
            <path
              d="M12 21s7-6.1 7-12a7 7 0 1 0-14 0c0 5.9 7 12 7 12Z"
              stroke="currentColor"
              stroke-width="1.8"
              stroke-linecap="round"
              stroke-linejoin="round"
            />
            <circle
              cx="12"
              cy="9"
              r="2.5"
              stroke="currentColor"
              stroke-width="1.8"
            />
          </svg>
        </span>
        Google Map's Location
      </h2>

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
  </section>

  <section aria-labelledby="mpuHeading">
    <div class="section-heading">
      <h2 id="mpuHeading" class="heading-with-icon">
        <span class="section-icon" aria-hidden="true">
          <svg viewBox="0 0 24 24" fill="none">
            <circle
              cx="5"
              cy="17"
              r="3.5"
              stroke="currentColor"
              stroke-width="1.7"
            />
            <circle
              cx="19"
              cy="17"
              r="3.5"
              stroke="currentColor"
              stroke-width="1.7"
            />
            <path
              d="M5 17 9 11h5l5 6M9 11l4 6H5M13 17l3-9h3M16 8l-2-3h-3M7 9h4"
              stroke="currentColor"
              stroke-width="1.7"
              stroke-linecap="round"
              stroke-linejoin="round"
            />
          </svg>
        </span>
        Motorcyle Positions
      </h2>

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
  </section>

  <footer>
    Last successful response:
    <span id="lastResponse">Not yet received</span>
  </footer>
</main>

<script>
const GPS_FRESHNESS_MS = 5000;
const byId = id => document.getElementById(id);

let latest = null;
let espReachable = false;
let lastResponseTime = 0;

function status(id, text, kind) {
  const element = byId(id);
  element.textContent = text;
  element.className = 'status ' + kind;
}

function usableLocation(data) {
  return !!data &&
    data.valid === true &&
    Number.isFinite(data.latitude) &&
    Number.isFinite(data.longitude) &&
    data.latitude >= -90 &&
    data.latitude <= 90 &&
    data.longitude >= -180 &&
    data.longitude <= 180;
}

function freshLocation() {
  if (
    !espReachable ||
    !usableLocation(latest) ||
    latest.fresh !== true ||
    !Number.isFinite(latest.age_ms) ||
    latest.age_ms < 0
  ) {
    return false;
  }

  const elapsed = performance.now() - lastResponseTime;

  return latest.age_ms + elapsed < GPS_FRESHNESS_MS;
}

function googleMapsURL(data) {
  const coordinates =
    data.latitude.toFixed(6) + ',' +
    data.longitude.toFixed(6);

  return 'https://www.google.com/maps/search/?api=1&query=' +
    encodeURIComponent(coordinates);
}

function disableMapsLink() {
  const link = byId('mapsLink');

  link.removeAttribute('href');
  link.setAttribute('aria-disabled', 'true');
  link.setAttribute('tabindex', '-1');

  byId('mapsURL').value = '';
}

function updateLocationActions() {
  const fresh = freshLocation();

  if (fresh) {
    const url = googleMapsURL(latest);
    const link = byId('mapsLink');

    link.href = url;
    link.setAttribute('aria-disabled', 'false');
    link.removeAttribute('tabindex');

    byId('mapsURL').value = url;

    status(
      'mapsStatus',
      'Google Maps link ready — using NEO-6M coordinates.',
      'good'
    );
  } else {
    disableMapsLink();

    if (!espReachable) {
      status(
        'mapsStatus',
        'ESP32 unreachable — Google Maps link disabled.',
        'bad'
      );
    } else if (usableLocation(latest)) {
      status(
        'mapsStatus',
        'GPS location is stale — waiting for fresh coordinates.',
        'warn'
      );
    } else {
      status(
        'mapsStatus',
        'Waiting for a valid GPS location.',
        'warn'
      );
    }
  }

  if (espReachable && usableLocation(latest) && !fresh) {
    status(
      'gpsStatus',
      'GPS location is stale — showing last-known coordinates',
      'warn'
    );
  }
}

function clearMPU() {
  ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temperature']
    .forEach(id => {
      byId(id).textContent = '';
    });
}

function renderData(data) {
  const locationOK = usableLocation(data);

  byId('latitude').textContent =
    locationOK ? data.latitude.toFixed(6) : '';

  byId('longitude').textContent =
    locationOK ? data.longitude.toFixed(6) : '';

  if (freshLocation()) {
    status(
      'gpsStatus',
      'GPS Location Available',
      'good'
    );
  } else if (locationOK) {
    status(
      'gpsStatus',
      'GPS location is stale — showing last-known coordinates',
      'warn'
    );
  } else if (data.receiving) {
    status(
      'gpsStatus',
      'GPS bytes received — waiting for a valid location',
      'warn'
    );
  } else {
    status(
      'gpsStatus',
      'No recent GPS bytes — check power, TX wiring and baud rate',
      'bad'
    );
  }

  const m = data.mpu;

  const keys = [
    'ax', 'ay', 'az',
    'gx', 'gy', 'gz',
    'temperature'
  ];

  if (!m || !m.detected) {
    clearMPU();

    status(
      'mpuStatus',
      'MPU6050 was not detected at startup. Check power, SDA, SCL ' +
      'and AD0, then restart the ESP32.',
      'bad'
    );
  } else if (
    !m.valid ||
    !keys.every(key => Number.isFinite(m[key]))
  ) {
    clearMPU();

    status(
      'mpuStatus',
      'MPU6050 reading unavailable.',
      'bad'
    );
  } else {
    keys.forEach(key => {
      byId(key).textContent = m[key].toFixed(2);
    });

    status(
      'mpuStatus',
      'MPU6050 readings available',
      'good'
    );
  }

  updateLocationActions();
}

byId('mapsLink').addEventListener('click', event => {
  if (!freshLocation()) {
    event.preventDefault();
    updateLocationActions();
    return;
  }

  byId('mapsLink').href = googleMapsURL(latest);
});

byId('mapsURL').addEventListener('click', () => {
  byId('mapsURL').select();
});

function unreachable() {
  espReachable = false;

  status(
    'gpsStatus',
    'ESP32 Is Unreachable. Please Check Your Connection To ESP32-GPS',
    'bad'
  );

  clearMPU();

  status(
    'mpuStatus',
    'ESP32 unreachable — MPU6050 readings unavailable.',
    'bad'
  );

  updateLocationActions();
}

async function poll() {
  const controller = new AbortController();

  const timeout = setTimeout(() => {
    controller.abort();
  }, 4000);

  try {
    const response = await fetch('/api/gps', {
      cache: 'no-store',
      signal: controller.signal
    });

    if (!response.ok) {
      throw new Error('HTTP error');
    }

    const data = await response.json();

    if (
      !data ||
      typeof data.valid !== 'boolean' ||
      typeof data.fresh !== 'boolean' ||
      typeof data.receiving !== 'boolean' ||
      !data.mpu ||
      typeof data.mpu.detected !== 'boolean' ||
      typeof data.mpu.valid !== 'boolean'
    ) {
      throw new Error('Invalid sensor response');
    }

    latest = data;
    espReachable = true;
    lastResponseTime = performance.now();

    renderData(data);

    byId('lastResponse').textContent =
      new Date().toLocaleTimeString();
  } catch (_) {
    unreachable();
  } finally {
    clearTimeout(timeout);

    // Sequential polling: next request starts after completion.
    setTimeout(poll, 1000);
  }
}

// Freshness checks only; no network requests from this timer.
setInterval(updateLocationActions, 250);

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