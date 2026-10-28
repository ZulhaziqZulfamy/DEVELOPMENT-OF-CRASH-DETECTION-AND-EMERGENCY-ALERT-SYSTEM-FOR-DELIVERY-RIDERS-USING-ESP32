#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ---------- Wiring summary (ESP32 DevKit) ----------
//
// NEO-6M GPS   TX  -> GPIO16 (ESP32 RX, UART1)
//              RX  -> GPIO17 (ESP32 TX, UART1)
// MPU6050      SDA -> GPIO21
//              SCL -> GPIO22
// SIM800L      TXD -> GPIO26 (ESP32 RX, UART2)
//              RXD -> GPIO27 (ESP32 TX, UART2) through a voltage divider
//              VCC -> separate 3.7-4.2 V supply, 2 A peak (NOT the ESP32 3V3 pin)
// Buzzer       +   -> GPIO25 (active buzzer, via NPN transistor if > 12 mA)
// All grounds must be connected together.
//
// Mount the MPU6050 flat with its X arrow pointing to the front of the vehicle,
// so roll = leaning left/right and pitch = nose up/down.

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

// ---------- Rider and emergency contact ----------

const char *RIDER_NAME = "Muhammad Irfan Bin Yusof";

// International format. Replace before use: no SMS is sent while it still contains 'X'.
const char *EMERGENCY_NUMBER = "+60XXXXXXXXX";

// ---------- Google Maps Static API ----------

// Google Cloud Console > APIs & Services > enable "Maps Static API" > Credentials > Create API key.
// Restrict the key to the Maps Static API: anyone who opens the dashboard can see it.
const char *GOOGLE_MAPS_API_KEY = "PASTE_YOUR_API_KEY_HERE";

// ---------- GPS configuration ----------

constexpr int GPS_RX_PIN = 16;
constexpr int GPS_TX_PIN = 17;
constexpr uint32_t GPS_BAUD = 9600;
constexpr uint32_t GPS_FRESHNESS_MS = 5000;
constexpr uint32_t GPS_STREAM_TIMEOUT_MS = 5000;

// ---------- MPU6050 configuration ----------

constexpr int MPU_SDA_PIN = 21;
constexpr int MPU_SCL_PIN = 22;
constexpr uint32_t MPU_SAMPLE_MS = 20;

// Complementary filter: trust the gyro for quick changes, the accelerometer to stop drift.
constexpr float ORIENTATION_GYRO_WEIGHT = 0.98f;

// ---------- Accident detection ----------

// Total acceleration (all three axes) that counts as a crash.
// A rider at rest reads about 1 g. Raise this if potholes trigger false alarms.
constexpr float CRASH_THRESHOLD_G = 2.5f;

constexpr int BUZZER_PIN = 25;
constexpr uint32_t BUZZER_DURATION_MS = 30000;
constexpr uint32_t BUZZER_BEEP_MS = 250;

// ---------- SIM800L configuration ----------

constexpr int GSM_RX_PIN = 26;
constexpr int GSM_TX_PIN = 27;
constexpr uint32_t GSM_BAUD = 9600;
constexpr uint32_t GSM_PROBE_INTERVAL_MS = 5000;

// UART1 explicitly mapped to GPIO16/GPIO17, UART2 to GPIO26/GPIO27.
HardwareSerial gpsSerial(1);
HardwareSerial gsmSerial(2);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;
WebServer server(80);

uint32_t lastGPSByteTime = 0;
bool receivedGPSBytes = false;
bool mpuDetected = false;

// Latest MPU6050 sample, refreshed every MPU_SAMPLE_MS by loop().
sensors_event_t accelEvent = {};
sensors_event_t gyroEvent = {};
sensors_event_t tempEvent = {};
bool mpuValid = false;
float currentG = 0.0f;
uint32_t lastMpuSample = 0;

// Vehicle orientation in degrees.
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
bool orientationReady = false;

// Crash alert state.
bool alertActive = false;
float alertImpactG = 0.0f;
uint32_t alertStartMs = 0;

// SMS sending runs as a small state machine so the web server never stalls.
enum SmsState {
  SMS_IDLE,
  SMS_WAIT_AT,
  SMS_WAIT_CMGF,
  SMS_WAIT_PROMPT,
  SMS_WAIT_RESULT,
  SMS_SENT,
  SMS_FAILED
};

SmsState smsState = SMS_IDLE;
uint32_t smsStepStart = 0;
uint32_t lastGsmProbe = 0;
bool gsmReady = false;
String gsmBuffer;
String smsBody;
String smsDetail;

// ---------- Embedded website ----------

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="theme-color" content="#7a3fe4">
<title>Rider Tracker</title>
<!-- Loaded without blocking the page: offline it falls back to the system fonts. -->
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" media="print" onload="this.media='all'"
      href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&family=JetBrains+Mono:wght@500;700;800&display=swap">
<style>
  :root {
    --brand: #7a3fe4;
    --brand-dark: #5a2bb8;
    --brand-soft: #efe7fd;
    --ink: #1e1a2b;
    --muted: #6f6a7d;
    --line: #e7e4ee;
    --bg: #f3f1f8;
    --surface: #ffffff;
    --green: #1f9d63;
    --green-soft: #e3f6ec;
    --red: #e5484d;
    --red-soft: #fde8e8;
    --amber: #c27806;
    --amber-soft: #fdf1dc;
    --blue: #2563c9;
    --blue-soft: #e4effc;
    --sans: "Inter", system-ui, -apple-system, "Segoe UI", Roboto, sans-serif;
    --mono: "JetBrains Mono", ui-monospace, "SF Mono", "Cascadia Mono", Consolas, monospace;
    color-scheme: light;
  }

  * { box-sizing: border-box; }

  html, body { height: 100%; margin: 0; }

  body {
    display: flex;
    flex-direction: column;
    background: var(--bg);
    color: var(--ink);
    font: 15px/1.45 var(--sans);
    -webkit-font-smoothing: antialiased;
  }

  button, input { font: inherit; }

  [hidden] { display: none !important; }

  /* ---------- Top bar ---------- */

  .topbar {
    flex: none;
    display: flex;
    align-items: center;
    gap: 16px;
    padding: 12px 24px;
    background: var(--surface);
    border-bottom: 1px solid var(--line);
  }

  .brand {
    display: flex;
    align-items: center;
    gap: 12px;
  }

  .brand-mark {
    display: grid;
    place-items: center;
    width: 38px;
    height: 38px;
    border-radius: 12px;
    background: linear-gradient(135deg, #a678f3, var(--brand));
    color: #fff;
  }

  .brand-mark svg { width: 22px; height: 22px; }
  .brand-name { font-size: 1.1rem; font-weight: 800; line-height: 1.2; }
  .brand-sub { font-size: 0.8rem; color: var(--muted); }

  .topbar-right {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-left: auto;
  }

  .updated {
    font-size: 0.82rem;
    color: var(--muted);
    font-variant-numeric: tabular-nums;
  }

  .pill {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    padding: 7px 14px;
    border-radius: 999px;
    border: 1px solid var(--line);
    background: #fff;
    font-size: 0.85rem;
    font-weight: 700;
    white-space: nowrap;
  }

  .pill::before {
    content: "";
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: currentColor;
  }

  .live-pill { color: var(--red); }
  .live-pill.on { color: var(--green); }
  .live-pill.on::before { animation: pulse 1.6s ease-in-out infinite; }

  .safety-pill { color: var(--green); background: var(--green-soft); border-color: transparent; }
  .safety-pill.alert { color: #fff; background: var(--red); animation: flash 1s infinite; }
  .safety-pill.alert::before { background: #fff; }

  /* ---------- Crash banner ---------- */

  .crash-banner {
    flex: none;
    display: flex;
    align-items: center;
    gap: 16px;
    padding: 12px 24px;
    background: var(--red);
    color: #fff;
  }

  .crash-banner svg { flex: none; width: 28px; height: 28px; animation: flash 1s infinite; }
  .crash-banner strong { font-size: 1.05rem; }
  .crash-banner span { opacity: 0.92; }

  .crash-banner .btn {
    width: auto;
    margin: 0 0 0 auto;
    padding: 0 20px;
    min-height: 40px;
    background: #fff;
    color: var(--red);
  }

  /* ---------- Layout ---------- */

  .layout {
    flex: 1;
    min-height: 0;
    display: grid;
    grid-template-columns: 340px minmax(0, 1fr) 360px;
    gap: 16px;
    padding: 16px 24px 24px;
  }

  .col {
    display: flex;
    flex-direction: column;
    gap: 16px;
    min-height: 0;
    overflow-y: auto;
    padding-right: 4px;
  }

  .card {
    padding: 18px;
    border: 1px solid var(--line);
    border-radius: 18px;
    background: var(--surface);
    box-shadow: 0 2px 10px rgba(30, 20, 60, 0.05);
  }

  .card-title {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    margin: 0 0 14px;
    font-size: 1.05rem;
    font-weight: 800;
  }

  .tag {
    padding: 3px 10px;
    border-radius: 999px;
    background: var(--brand-soft);
    color: var(--brand-dark);
    font-size: 0.72rem;
    font-weight: 700;
  }

  .sub-title {
    margin: 18px 0 8px;
    font-size: 0.92rem;
    font-weight: 800;
  }

  /* ---------- Map ---------- */

  .map-card {
    display: flex;
    flex-direction: column;
    min-height: 0;
    padding: 0;
    overflow: hidden;
  }

  .map-toolbar {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 12px 16px;
    border-bottom: 1px solid var(--line);
  }

  .map-toolbar h2 { margin: 0; font-size: 1.05rem; font-weight: 800; }

  .map-tools {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-left: auto;
  }

  .map-updated {
    font-size: 0.82rem;
    color: var(--muted);
    font-variant-numeric: tabular-nums;
  }

  .zoom {
    display: inline-flex;
    align-items: center;
    gap: 6px;
  }

  .zoom .btn-small {
    justify-content: center;
    width: 34px;
    padding: 6px 0;
    font-size: 1rem;
  }

  .zoom-level {
    min-width: 2.2em;
    text-align: center;
    font: 700 0.85rem/1 var(--mono);
    color: var(--muted);
  }

  .btn-small {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 7px 14px;
    border: 1px solid var(--line);
    border-radius: 999px;
    background: #fff;
    color: var(--brand-dark);
    font-size: 0.85rem;
    font-weight: 700;
    cursor: pointer;
  }

  .btn-small:hover { background: var(--brand-soft); }
  .btn-small:disabled { color: #9a94a8; background: #fff; cursor: default; }
  .btn-small svg { width: 16px; height: 16px; }

  .map {
    position: relative;
    flex: 1;
    min-height: 320px;
    background-color: #ebe7f4;
    background-image:
      linear-gradient(rgba(122, 63, 228, 0.08) 1px, transparent 1px),
      linear-gradient(90deg, rgba(122, 63, 228, 0.08) 1px, transparent 1px);
    background-size: 32px 32px;
  }

  .map img {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 100%;
    object-fit: cover;
  }

  .map-empty {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    padding: 24px;
  }

  .map-empty span {
    max-width: 440px;
    text-align: center;
    padding: 10px 18px;
    border-radius: 999px;
    background: #fff;
    color: var(--muted);
    font-weight: 600;
    box-shadow: 0 4px 14px rgba(30, 20, 60, 0.12);
  }

  .map-note {
    padding: 8px 16px;
    border-top: 1px solid var(--line);
    font-size: 0.78rem;
    color: var(--muted);
  }

  /* ---------- Rider ---------- */

  .member {
    display: flex;
    align-items: center;
    gap: 12px;
  }

  .avatar {
    flex: none;
    display: grid;
    place-items: center;
    width: 52px;
    height: 52px;
    border-radius: 50%;
    background: linear-gradient(135deg, #a678f3, var(--brand));
    color: #fff;
    font-weight: 800;
    box-shadow: 0 0 0 3px #fff, 0 0 0 5px var(--brand);
  }

  .avatar.stale { background: #b8b2c6; box-shadow: 0 0 0 3px #fff, 0 0 0 5px #9a94a8; }
  .avatar.crash { background: var(--red); box-shadow: 0 0 0 3px #fff, 0 0 0 5px var(--red); animation: flash 1s infinite; }

  .member-text { min-width: 0; }
  .member-role { font-size: 0.78rem; font-weight: 700; color: var(--brand); }
  .member-name { font-weight: 800; }
  .member-sub { font-size: 0.85rem; color: var(--muted); }

  .status {
    --tone: var(--blue);
    --tone-bg: var(--blue-soft);
    display: flex;
    align-items: center;
    gap: 10px;
    margin: 14px 0;
    padding: 10px 12px;
    border-radius: 12px;
    background: var(--tone-bg);
    color: var(--tone);
    font-size: 0.88rem;
    font-weight: 600;
  }

  .status::before {
    content: "";
    flex: none;
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: var(--tone);
  }

  .status.info  { --tone: var(--blue);  --tone-bg: var(--blue-soft); }
  .status.ok    { --tone: var(--green); --tone-bg: var(--green-soft); }
  .status.warn  { --tone: var(--amber); --tone-bg: var(--amber-soft); }
  .status.error { --tone: var(--red);   --tone-bg: var(--red-soft); }

  .grid2 {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 10px;
  }

  .tile {
    padding: 12px;
    border: 1px solid var(--line);
    border-radius: 14px;
    background: var(--bg);
  }

  .label {
    display: block;
    font-size: 0.72rem;
    font-weight: 700;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: var(--muted);
  }

  .value {
    margin: 2px 0;
    font: 700 1.1rem/1.3 var(--mono);
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
  }

  .value.stale { color: var(--amber); }

  .note {
    font-size: 0.75rem;
    font-weight: 400;
    color: var(--muted);
  }

  /* ---------- Buttons and fields ---------- */

  .btn {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    width: 100%;
    min-height: 46px;
    margin: 12px 0;
    border: 0;
    border-radius: 999px;
    background: var(--brand);
    color: #fff;
    font-weight: 700;
    text-decoration: none;
    cursor: pointer;
    transition: background-color 0.15s ease;
  }

  .btn svg { width: 20px; height: 20px; }
  .btn:hover { background: var(--brand-dark); }
  .btn.danger { background: var(--red); }
  .btn.danger:hover { background: #c93a3f; }

  .btn[aria-disabled="true"],
  .btn:disabled {
    background: #e4e0ec;
    color: #9a94a8;
    pointer-events: none;
  }

  .url-row {
    display: flex;
    gap: 8px;
    margin-top: 6px;
  }

  .url-field {
    flex: 1;
    min-width: 0;
    padding: 10px 12px;
    border: 1px solid var(--line);
    border-radius: 12px;
    background: var(--bg);
    color: var(--ink);
    font: 0.8rem/1.4 var(--mono);
  }

  .helper-text {
    margin: 10px 0 0;
    font-size: 0.82rem;
    color: var(--muted);
  }

  /* ---------- Lists ---------- */

  .list {
    border: 1px solid var(--line);
    border-radius: 14px;
    overflow: hidden;
  }

  .row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
    padding: 10px 14px;
    border-top: 1px solid var(--line);
  }

  .row:first-child { border-top: 0; }
  .row > span:first-child { color: var(--muted); }
  .row strong { text-align: right; overflow-wrap: anywhere; }

  .axis-name {
    display: inline-flex;
    align-items: center;
    gap: 8px;
  }

  .axis-name::before {
    content: "";
    width: 10px;
    height: 10px;
    border-radius: 3px;
    background: var(--axis);
  }

  .axis-value {
    font: 700 1rem/1.2 var(--mono);
    font-variant-numeric: tabular-nums;
  }

  .axis-x { --axis: #e5484d; }
  .axis-y { --axis: #1f9d63; }
  .axis-z { --axis: #2563c9; }

  .axis-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
  }

  .axis-grid .sub-title { margin-top: 0; }

  /* ---------- Driving ---------- */

  .attitude-grid {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
  }

  .attitude {
    padding: 14px 12px 12px;
    border-radius: 16px;
    background: var(--brand-soft);
    text-align: center;
  }

  .dial {
    display: block;
    width: 120px;
    height: 120px;
    margin: 8px auto 6px;
  }

  .vehicle {
    transform-box: view-box;
    transition: transform 0.25s ease-out;
  }

  #rollVehicle { transform-origin: 60px 96px; }
  #pitchVehicle { transform-origin: 60px 84px; }

  .attitude-value {
    font: 800 1.5rem/1.1 var(--mono);
    font-variant-numeric: tabular-nums;
    color: var(--brand-dark);
  }

  .attitude-dir {
    font-size: 0.8rem;
    font-weight: 600;
    color: var(--muted);
  }

  /* ---------- Safety ---------- */

  .safety-hero {
    display: flex;
    align-items: center;
    gap: 14px;
    padding: 16px;
    border-radius: 16px;
    background: var(--green-soft);
  }

  .safety-icon {
    flex: none;
    display: grid;
    place-items: center;
    width: 48px;
    height: 48px;
    border-radius: 50%;
    background: var(--green);
    color: #fff;
  }

  .safety-icon svg { width: 26px; height: 26px; }
  .safety-title { font-size: 1.1rem; font-weight: 800; color: var(--green); }
  .safety-sub { font-size: 0.85rem; color: var(--muted); }

  .safety-hero.alert { background: var(--red-soft); }
  .safety-hero.alert .safety-icon { background: var(--red); animation: flash 1s infinite; }
  .safety-hero.alert .safety-title { color: var(--red); }

  .meter-head {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
    margin-top: 16px;
  }

  .meter-head strong {
    font: 800 1.3rem/1 var(--mono);
    font-variant-numeric: tabular-nums;
  }

  .meter {
    position: relative;
    height: 14px;
    margin: 8px 0 4px;
    border-radius: 7px;
    background: #eeeaf5;
    overflow: hidden;
  }

  .meter-fill {
    position: absolute;
    top: 0;
    bottom: 0;
    left: 0;
    width: 0;
    background: var(--green);
    transition: width 0.3s ease-out, background-color 0.3s;
  }

  .meter-mark {
    position: absolute;
    top: 0;
    bottom: 0;
    left: 62.5%;
    width: 3px;
    background: var(--ink);
  }

  .meter-scale {
    position: relative;
    height: 1.4em;
    font-size: 0.75rem;
    color: var(--muted);
  }

  .meter-scale span { position: absolute; }
  .meter-scale span:last-child { right: 0; }

  #thresholdLabel {
    left: 62.5%;
    transform: translateX(-50%);
    font-weight: 700;
    color: var(--ink);
  }

  a:focus-visible,
  button:focus-visible,
  input:focus-visible {
    outline: 3px solid var(--brand);
    outline-offset: 2px;
  }

  @keyframes pulse {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.35; }
  }

  @keyframes flash {
    0%, 100% { filter: none; }
    50% { filter: brightness(1.25); }
  }

  /* Smaller windows: map on top, panels below, whole page scrolls. */
  @media (max-width: 1180px) {
    body { height: auto; display: block; }
    .layout { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .map-card { grid-column: 1 / -1; grid-row: 1; height: 460px; }
    .col { overflow: visible; padding-right: 0; }
  }

  @media (max-width: 760px) {
    .layout { grid-template-columns: 1fr; padding: 12px; }
    .topbar { flex-wrap: wrap; padding: 12px; }
    .crash-banner { flex-wrap: wrap; padding: 12px; }
    .crash-banner .btn { margin-left: 0; }
  }

  @media (prefers-reduced-motion: reduce) {
    * { animation: none !important; transition: none !important; }
  }
</style>
</head>
<body>
  <header class="topbar">
    <div class="brand">
      <div class="brand-mark" aria-hidden="true">
        <svg viewBox="0 0 24 24" fill="none">
          <path d="M12 21s-7-6.2-7-11.5A7 7 0 0 1 19 9.5C19 14.8 12 21 12 21Z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/>
          <circle cx="12" cy="9.5" r="2.5" stroke="currentColor" stroke-width="2"/>
        </svg>
      </div>
      <div>
        <div class="brand-name">Delivery Riders</div>
        <div class="brand-sub">Rider Tracker · Data Is Sourced By ESP32</div>
      </div>
    </div>

    <div class="topbar-right">
      <span id="updated" class="updated">Waiting for data…</span>
      <span id="safetyPill" class="pill safety-pill">Crash detection on</span>
      <span id="livePill" class="pill live-pill">Connecting</span>
    </div>
  </header>

  <div id="crashBanner" class="crash-banner" role="alert" hidden>
    <svg viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path d="M12 3 4 6v6c0 4.5 3.4 8.3 8 9 4.6-.7 8-4.5 8-9V6l-8-3Z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/>
      <path d="M12 8v5M12 16h.01" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"/>
    </svg>
    <div>
      <strong>Crash detected · Muhammad Irfan Bin Yusof</strong><br>
      <span id="crashBannerText">Impact — g</span>
    </div>
    <button class="btn reset-alert" type="button">I'm safe · Reset alert</button>
  </div>

  <main class="layout">
    <!-- ---------- Left column: rider and location ---------- -->
    <aside class="col">
      <section class="card" aria-labelledby="riderHeading">
        <h2 id="riderHeading" class="card-title">Rider <span class="tag">NEO-6M GPS</span></h2>

        <div class="member">
          <div id="avatar" class="avatar" aria-hidden="true">MI</div>
          <div class="member-text">
            <div class="member-role">Delivery Rider 1</div>
            <div class="member-name">Muhammad Irfan Bin Yusof</div>
            <div id="memberSub" class="member-sub">Locating…</div>
          </div>
        </div>

        <div id="gpsStatus" class="status info" role="status">Connecting to ESP32…</div>

        <div class="grid2">
          <div class="tile">
            <span class="label">Latitude</span>
            <div id="latitude" class="value">—</div>
            <div class="note">Decimal Degrees</div>
          </div>
          <div class="tile">
            <span class="label">Longitude</span>
            <div id="longitude" class="value">—</div>
            <div class="note">Decimal Degrees</div>
          </div>
        </div>
      </section>

      <section class="card" aria-labelledby="mapsHeading">
        <h2 id="mapsHeading" class="card-title">Google Maps <span class="tag">External URL Link</span></h2>

        <div id="mapsStatus" class="status info" role="status">Waiting for fresh GPS coordinates.</div>

        <a id="mapsLink" class="btn" target="_blank" rel="noopener noreferrer" aria-disabled="true" tabindex="-1">
          <svg viewBox="0 0 24 24" fill="none" aria-hidden="true">
            <path d="M14 3h7v7M21 3 10 14M10 3H5a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-5"
                  stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>
          </svg>
          Open in Google Maps
        </a>

        <label class="label" for="mapsURL">Location URL</label>
        <div class="url-row">
          <input id="mapsURL" class="url-field" type="text" readonly
                 placeholder="Available when GPS coordinates are fresh"
                 aria-label="Google Maps URL from NEO-6M coordinates">
          <button id="copyURL" class="btn-small" type="button" disabled>Copy</button>
        </div>

        <p class="helper-text">
          Opens the NEO-6M's reported coordinates in a new tab.
          The PC needs Internet access to load Google Maps.
        </p>
      </section>
    </aside>

    <!-- ---------- Centre: Google Maps Static API map ---------- -->
    <section class="card map-card" aria-labelledby="mapHeading">
      <div class="map-toolbar">
        <h2 id="mapHeading">Live map</h2>
        <span class="tag">Maps Static API</span>
        <div class="map-tools">
          <span id="mapUpdated" class="map-updated">Not loaded yet</span>
          <div class="zoom" role="group" aria-label="Map zoom">
            <button id="zoomOut" class="btn-small" type="button" aria-label="Zoom out">−</button>
            <span id="zoomLevel" class="zoom-level">17</span>
            <button id="zoomIn" class="btn-small" type="button" aria-label="Zoom in">+</button>
          </div>
          <button id="refreshMap" class="btn-small" type="button" disabled>
            <svg viewBox="0 0 24 24" fill="none" aria-hidden="true">
              <path d="M20 11a8 8 0 1 0-2.3 5.7M20 4v7h-7" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
            </svg>
            Refresh
          </button>
        </div>
      </div>

      <div class="map">
        <img id="mapImg" alt="Google map of the rider's location" hidden>
        <div id="mapEmpty" class="map-empty"><span id="mapMsg">Waiting for GPS fix…</span></div>
      </div>

      <div class="map-note">
        Map image from the Google Maps Static API, refreshed every 5 seconds while the rider has a GPS fix.
        Use "Open in Google Maps" to explore the area.
      </div>
    </section>

    <!-- ---------- Right column: safety and driving ---------- -->
    <aside class="col">
      <section class="card" aria-labelledby="safetyHeading">
        <h2 id="safetyHeading" class="card-title">Crash detection <span class="tag">MPU6050</span></h2>

        <div id="safetyHero" class="safety-hero" role="status">
          <div class="safety-icon" aria-hidden="true">
            <svg viewBox="0 0 24 24" fill="none">
              <path d="M12 3 4 6v6c0 4.5 3.4 8.3 8 9 4.6-.7 8-4.5 8-9V6l-8-3Z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/>
            </svg>
          </div>
          <div>
            <div id="safetyTitle" class="safety-title">Rider is safe</div>
            <div id="safetySub" class="safety-sub">Monitoring the MPU6050 for impacts.</div>
          </div>
        </div>

        <div class="meter-head">
          <span class="label">Impact force now</span>
          <strong><span id="gNow">—</span> g</strong>
        </div>
        <div class="meter" role="img" aria-label="Impact force compared to the crash threshold">
          <div id="gFill" class="meter-fill"></div>
          <div id="gMark" class="meter-mark"></div>
        </div>
        <div class="meter-scale">
          <span>0 g</span>
          <span id="thresholdLabel">2.5 g</span>
          <span>4 g</span>
        </div>

        <h3 class="sub-title">Emergency response</h3>
        <div class="list">
          <div class="row"><span>Buzzer</span><strong id="buzzerState">Off</strong></div>
          <div class="row"><span>SMS alert</span><strong id="smsState">Standing by</strong></div>
          <div class="row"><span>Emergency contact</span><strong id="contact">—</strong></div>
          <div class="row"><span>GSM module</span><strong id="gsmState">—</strong></div>
        </div>

        <button id="resetAlert" class="btn danger reset-alert" type="button" disabled>I'm safe · Reset alert</button>
      </section>

      <section class="card" aria-labelledby="drivingHeading">
        <h2 id="drivingHeading" class="card-title">Driving <span class="tag">MPU6050 Accelerometer</span></h2>

        <h3 class="sub-title" style="margin-top:0">Live vehicle orientation</h3>
        <div class="attitude-grid">
          <div class="attitude">
            <span class="label">Lean (roll)</span>
            <svg class="dial" viewBox="0 0 120 120" role="img" aria-label="Rear view of the vehicle showing its lean">
              <circle cx="60" cy="60" r="58" fill="#fff" stroke="#dcd0f7" stroke-width="2"/>
              <path d="M2 96h116" stroke="#cbbcf2" stroke-width="2"/>
              <g stroke="#b9a6ee" stroke-width="2" stroke-linecap="round">
                <path d="M60 4v8"/>
                <path d="M32 11.5l4 6.9"/>
                <path d="M88 11.5l-4 6.9"/>
              </g>
              <g id="rollVehicle" class="vehicle">
                <rect x="54" y="62" width="12" height="34" rx="6" fill="#1e1a2b"/>
                <rect x="47" y="52" width="26" height="14" rx="5" fill="#7a3fe4"/>
                <path d="M34 46h52" stroke="#1e1a2b" stroke-width="4" stroke-linecap="round"/>
                <rect x="52" y="34" width="16" height="18" rx="7" fill="#5a2bb8"/>
                <circle cx="60" cy="25" r="8" fill="#5a2bb8"/>
              </g>
            </svg>
            <div id="rollValue" class="attitude-value">—</div>
            <div id="rollDir" class="attitude-dir">Waiting…</div>
          </div>

          <div class="attitude">
            <span class="label">Pitch</span>
            <svg class="dial" viewBox="0 0 120 120" role="img" aria-label="Side view of the vehicle showing its pitch">
              <circle cx="60" cy="60" r="58" fill="#fff" stroke="#dcd0f7" stroke-width="2"/>
              <path d="M2 96h116" stroke="#cbbcf2" stroke-width="2"/>
              <g id="pitchVehicle" class="vehicle">
                <circle cx="35" cy="84" r="12" fill="none" stroke="#1e1a2b" stroke-width="4"/>
                <circle cx="85" cy="84" r="12" fill="none" stroke="#1e1a2b" stroke-width="4"/>
                <path d="M35 84l14-20h24l12 20" fill="none" stroke="#7a3fe4" stroke-width="5" stroke-linejoin="round"/>
                <path d="M73 64l5-14h7" fill="none" stroke="#1e1a2b" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>
                <path d="M56 62l6-16" stroke="#5a2bb8" stroke-width="8" stroke-linecap="round"/>
                <circle cx="64" cy="35" r="7" fill="#5a2bb8"/>
              </g>
            </svg>
            <div id="pitchValue" class="attitude-value">—</div>
            <div id="pitchDir" class="attitude-dir">Waiting…</div>
          </div>
        </div>

        <div id="orientStatus" class="status info" role="status">Waiting for orientation data…</div>

        <div id="mpuStatus" class="status info" role="status">Waiting for MPU6050 data…</div>

        <div class="axis-grid">
          <div>
            <h3 class="sub-title">Acceleration <span class="note">m/s²</span></h3>
            <div class="list">
              <div class="row"><span class="axis-name axis-x">X</span><span id="ax" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-y">Y</span><span id="ay" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-z">Z</span><span id="az" class="axis-value">—</span></div>
            </div>
          </div>
          <div>
            <h3 class="sub-title">Angular velocity <span class="note">rad/s</span></h3>
            <div class="list">
              <div class="row"><span class="axis-name axis-x">X</span><span id="gx" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-y">Y</span><span id="gy" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-z">Z</span><span id="gz" class="axis-value">—</span></div>
            </div>
          </div>
        </div>
      </section>
    </aside>
  </main>

<script>
  const POLL_MS = 1000;
  const MAP_REFRESH_MS = 5000;
  const METER_MAX_G = 4;
  const $ = (id) => document.getElementById(id);

  // The ESP32 swaps in GOOGLE_MAPS_API_KEY when it serves this page.
  const MAPS_KEY = '%MAPS_KEY%';
  const HAS_MAPS_KEY = MAPS_KEY.length > 0 && MAPS_KEY.indexOf('MAPS_KEY') < 0 && MAPS_KEY.indexOf('PASTE_') !== 0;

  let lastFix = null;
  let fixSince = null;
  let mapZoom = 17;
  let lastMapUrl = '';
  let alertActive = false;

  function setStatus(id, tone, text) {
    const el = $(id);
    el.className = 'status ' + tone;
    el.textContent = text;
  }

  function fmt(value, digits) {
    return (typeof value === 'number' && isFinite(value)) ? value.toFixed(digits) : '—';
  }

  function clock(ms, seconds) {
    const options = { hour: 'numeric', minute: '2-digit' };
    if (seconds) options.second = '2-digit';
    return new Date(ms).toLocaleTimeString([], options);
  }

  function showMapMessage(text) {
    $('mapMsg').textContent = text;
    $('mapEmpty').hidden = false;
  }

  function refreshMap(force) {
    if (!lastFix) return;

    if (!HAS_MAPS_KEY) {
      showMapMessage('Add your Google Maps Static API key to GOOGLE_MAPS_API_KEY in the sketch to show the map.');
      return;
    }

    // The Static API allows up to 640×640; scale=2 keeps it sharp on high-DPI screens.
    const box = $('mapImg').parentElement.getBoundingClientRect();
    const width = Math.max(100, Math.min(640, Math.round(box.width)));
    const height = Math.max(100, Math.min(640, Math.round(box.height)));
    const point = lastFix[0].toFixed(6) + ',' + lastFix[1].toFixed(6);
    const colour = alertActive ? 'red' : '0x7a3fe4';

    const url = 'https://maps.googleapis.com/maps/api/staticmap?center=' + point +
                '&zoom=' + mapZoom + '&size=' + width + 'x' + height + '&scale=2' +
                '&markers=color:' + colour + '%7Clabel:M%7C' + point +
                '&key=' + encodeURIComponent(MAPS_KEY);

    // Same picture as last time: skip the request so the API quota isn't spent on nothing.
    if (url === lastMapUrl && !force) return;

    lastMapUrl = url;
    $('mapImg').src = url;
  }

  $('mapImg').addEventListener('load', () => {
    $('mapImg').hidden = false;
    $('mapEmpty').hidden = true;
    $('mapUpdated').textContent = 'Map updated ' + clock(Date.now(), true);
  });

  $('mapImg').addEventListener('error', () => {
    $('mapImg').hidden = true;
    lastMapUrl = '';
    showMapMessage("The map image didn't load. Check the PC has Internet, the API key is correct " +
                   'and the Maps Static API is enabled for it.');
  });

  function setZoom(zoom) {
    mapZoom = Math.min(20, Math.max(3, zoom));
    $('zoomLevel').textContent = mapZoom;
    refreshMap(true);
  }

  $('zoomIn').addEventListener('click', () => setZoom(mapZoom + 1));
  $('zoomOut').addEventListener('click', () => setZoom(mapZoom - 1));
  $('refreshMap').addEventListener('click', () => refreshMap(true));

  setInterval(() => refreshMap(false), MAP_REFRESH_MS);

  $('copyURL').addEventListener('click', async () => {
    const field = $('mapsURL');
    if (!field.value) return;

    // The ESP32 page is plain http, where the Clipboard API is usually unavailable.
    try {
      await navigator.clipboard.writeText(field.value);
    } catch (error) {
      field.select();
      document.execCommand('copy');
    }

    $('copyURL').textContent = 'Copied';
    setTimeout(() => { $('copyURL').textContent = 'Copy'; }, 1500);
  });

  document.querySelectorAll('.reset-alert').forEach((button) => {
    button.addEventListener('click', async () => {
      document.querySelectorAll('.reset-alert').forEach((b) => { b.disabled = true; });
      try {
        await fetch('/api/alert/reset', { method: 'POST' });
      } catch (error) {
        // The next poll shows whether the reset reached the ESP32.
      }
    });
  });

  function updateMap(d) {
    if (!d.valid) {
      if (!lastFix) showMapMessage('Waiting for GPS fix…');
      return;
    }

    const firstFix = !lastFix;
    lastFix = [d.latitude, d.longitude];
    $('refreshMap').disabled = false;

    // Show the map straight away on the first fix; after that the 5-second timer refreshes it.
    if (firstFix) refreshMap(true);
  }

  function updateGPS(d) {
    const lat = $('latitude');
    const lng = $('longitude');

    lat.textContent = d.valid ? fmt(d.latitude, 6) : '—';
    lng.textContent = d.valid ? fmt(d.longitude, 6) : '—';
    lat.classList.toggle('stale', d.valid && !d.fresh);
    lng.classList.toggle('stale', d.valid && !d.fresh);
    $('avatar').classList.toggle('stale', !d.fresh);

    if (d.fresh) {
      if (!fixSince) fixSince = Date.now();
    } else {
      fixSince = null;
    }

    if (!d.receiving) {
      setStatus('gpsStatus', 'error', 'No data from the GPS module. Check wiring (GPS TX → GPIO16).');
      $('memberSub').textContent = 'GPS offline';
    } else if (!d.valid) {
      setStatus('gpsStatus', 'warn', 'GPS connected, waiting for a satellite fix…');
      $('memberSub').textContent = 'Locating…';
    } else if (!d.fresh) {
      setStatus('gpsStatus', 'warn', 'GPS fix is stale (last update ' + Math.round(d.age_ms / 1000) + ' s ago).');
      $('memberSub').textContent = 'Last known location';
    } else {
      setStatus('gpsStatus', 'ok', 'Live GPS fix');
      $('memberSub').textContent = 'Live location · since ' + clock(fixSince, false);
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
      $('copyURL').disabled = false;
      setStatus('mapsStatus', 'ok', 'Location link ready.');
    } else {
      link.removeAttribute('href');
      link.setAttribute('aria-disabled', 'true');
      link.setAttribute('tabindex', '-1');
      field.value = '';
      $('copyURL').disabled = true;
      setStatus('mapsStatus', 'info', 'Waiting for fresh GPS coordinates.');
    }

    updateMap(d);
  }

  function updateMPU(m) {
    if (!m || !m.detected) {
      setStatus('mpuStatus', 'error', 'MPU6050 not detected. Check SDA → GPIO21 and SCL → GPIO22.');
    } else if (!m.valid) {
      setStatus('mpuStatus', 'warn', 'MPU6050 detected, but the last reading failed.');
    } else {
      setStatus('mpuStatus', 'ok', 'Live MPU6050 data');
    }

    const valid = m && m.valid;
    ['ax', 'ay', 'az', 'gx', 'gy', 'gz'].forEach((id) => {
      $(id).textContent = valid ? fmt(m[id], 2) : '—';
    });

    updateOrientation(valid ? m.roll : null, valid ? m.pitch : null);
  }

  function updateOrientation(roll, pitch) {
    if (typeof roll !== 'number' || typeof pitch !== 'number') {
      ['rollValue', 'pitchValue'].forEach((id) => { $(id).textContent = '—'; });
      ['rollDir', 'pitchDir'].forEach((id) => { $(id).textContent = 'Waiting…'; });
      setStatus('orientStatus', 'info', 'Waiting for orientation data…');
      return;
    }

    // SVG rotates clockwise for positive angles: lean right turns the rear view clockwise,
    // nose up turns the side view anticlockwise.
    $('rollVehicle').style.transform = 'rotate(' + Math.max(-90, Math.min(90, roll)) + 'deg)';
    $('pitchVehicle').style.transform = 'rotate(' + Math.max(-90, Math.min(90, -pitch)) + 'deg)';

    $('rollValue').textContent = Math.abs(roll).toFixed(0) + '°';
    $('pitchValue').textContent = Math.abs(pitch).toFixed(0) + '°';
    $('rollDir').textContent = Math.abs(roll) < 3 ? 'Level' : (roll > 0 ? 'Leaning right' : 'Leaning left');
    $('pitchDir').textContent = Math.abs(pitch) < 3 ? 'Level' : (pitch > 0 ? 'Nose up' : 'Nose down');

    const lean = Math.max(Math.abs(roll), Math.abs(pitch));

    if (lean >= 60) {
      setStatus('orientStatus', 'error', 'Vehicle has tipped over.');
    } else if (lean >= 30) {
      setStatus('orientStatus', 'warn', 'Steep angle: ' + lean.toFixed(0) + '°.');
    } else {
      setStatus('orientStatus', 'ok', 'Vehicle is upright.');
    }
  }

  function updateAlert(a, m) {
    const threshold = a.threshold_g;
    const markPercent = Math.min(threshold / METER_MAX_G, 1) * 100 + '%';
    $('gMark').style.left = markPercent;
    $('thresholdLabel').style.left = markPercent;
    $('thresholdLabel').textContent = fmt(threshold, 1) + ' g';

    const g = (m && m.valid) ? m.g : null;
    $('gNow').textContent = fmt(g, 2);

    const fill = $('gFill');
    fill.style.width = g === null ? '0' : Math.min(g / METER_MAX_G, 1) * 100 + '%';
    fill.style.background = g === null || g < threshold * 0.6 ? 'var(--green)'
                          : g < threshold ? 'var(--amber)' : 'var(--red)';

    // Turns the map marker red at the next map refresh.
    alertActive = a.active;

    const buzzerText = a.buzzer ? 'Sounding' : (a.active ? 'Stopped' : 'Off');
    const smsText = { idle: 'Standing by', sending: 'Sending…', sent: 'Sent', failed: 'Failed' }[a.sms] || a.sms;

    $('safetyHero').classList.toggle('alert', a.active);
    $('safetyPill').classList.toggle('alert', a.active);
    $('avatar').classList.toggle('crash', a.active);
    $('crashBanner').hidden = !a.active;
    document.querySelectorAll('.reset-alert').forEach((b) => { b.disabled = !a.active; });

    if (a.active) {
      const impact = 'Impact ' + fmt(a.impact_g, 1) + ' g · ' + a.seconds_ago + ' s ago';
      $('safetyTitle').textContent = 'Crash detected';
      $('safetySub').textContent = impact;
      $('safetyPill').textContent = 'Crash detected';
      $('crashBannerText').textContent = impact + ' · Buzzer ' + buzzerText.toLowerCase() +
                                         ' · SMS ' + smsText.toLowerCase();
    } else {
      $('safetyTitle').textContent = 'Rider is safe';
      $('safetySub').textContent = 'Monitoring the MPU6050 for impacts.';
      $('safetyPill').textContent = 'Crash detection on';
    }

    $('buzzerState').textContent = buzzerText;
    $('smsState').textContent = a.sms_detail ? smsText + ' · ' + a.sms_detail : smsText;
    $('contact').textContent = a.contact || '—';
    $('gsmState').textContent = a.gsm_ready ? 'SIM800L ready' : 'Not responding';
  }

  function showOffline() {
    $('livePill').className = 'pill live-pill';
    $('livePill').textContent = 'Offline';
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

      $('livePill').className = 'pill live-pill on';
      $('livePill').textContent = 'ESP32 live';
      $('updated').textContent = 'Updated ' + clock(Date.now(), true);

      updateGPS(data);
      updateMPU(data.mpu);
      updateAlert(data.alert, data.mpu);
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

// ---------- SIM800L SMS ----------

void readGSM() {
  while (gsmSerial.available() > 0) {
    const int value = gsmSerial.read();

    if (value < 0) {
      break;
    }

    // Keep only the most recent replies.
    if (gsmBuffer.length() >= 256) {
      gsmBuffer.remove(0, 128);
    }

    gsmBuffer += static_cast<char>(value);
  }
}

void gsmCommand(const char *command) {
  gsmBuffer = "";
  gsmSerial.print(command);
  gsmSerial.print("\r");
  smsStepStart = millis();
}

bool smsBusy() {
  return smsState == SMS_WAIT_AT ||
         smsState == SMS_WAIT_CMGF ||
         smsState == SMS_WAIT_PROMPT ||
         smsState == SMS_WAIT_RESULT;
}

void smsFail(const char *reason) {
  smsState = SMS_FAILED;
  smsDetail = reason;
}

const char *smsStateName() {
  switch (smsState) {
    case SMS_IDLE:   return "idle";
    case SMS_SENT:   return "sent";
    case SMS_FAILED: return "failed";
    default:         return "sending";
  }
}

void startSMS() {
  if (smsBusy()) {
    return;
  }

  if (strchr(EMERGENCY_NUMBER, 'X') != nullptr) {
    smsFail("Set EMERGENCY_NUMBER in the sketch");
    return;
  }

  smsDetail = "";
  smsState = SMS_WAIT_AT;
  gsmCommand("AT");
}

void updateSMS() {
  readGSM();

  const uint32_t elapsed = millis() - smsStepStart;

  switch (smsState) {
    case SMS_WAIT_AT:
      if (gsmBuffer.indexOf("OK") >= 0) {
        gsmReady = true;
        smsState = SMS_WAIT_CMGF;
        gsmCommand("AT+CMGF=1");
      } else if (elapsed > 3000) {
        gsmReady = false;
        smsFail("No response from SIM800L");
      }
      break;

    case SMS_WAIT_CMGF:
      if (gsmBuffer.indexOf("OK") >= 0) {
        gsmBuffer = "";
        gsmSerial.print("AT+CMGS=\"");
        gsmSerial.print(EMERGENCY_NUMBER);
        gsmSerial.print("\"\r");
        smsStepStart = millis();
        smsState = SMS_WAIT_PROMPT;
      } else if (gsmBuffer.indexOf("ERROR") >= 0 || elapsed > 3000) {
        smsFail("SIM800L rejected text mode");
      }
      break;

    case SMS_WAIT_PROMPT:
      if (gsmBuffer.indexOf('>') >= 0) {
        gsmBuffer = "";
        gsmSerial.print(smsBody);
        gsmSerial.write(26);  // Ctrl+Z sends the message.
        smsStepStart = millis();
        smsState = SMS_WAIT_RESULT;
      } else if (gsmBuffer.indexOf("ERROR") >= 0 || elapsed > 5000) {
        gsmSerial.write(27);  // Esc cancels the half-started message.
        smsFail("No SMS prompt (check SIM card and signal)");
      }
      break;

    case SMS_WAIT_RESULT:
      if (gsmBuffer.indexOf("+CMGS:") >= 0) {
        smsState = SMS_SENT;
        smsDetail = "";
      } else if (gsmBuffer.indexOf("ERROR") >= 0 || elapsed > 60000) {
        smsFail("Network did not accept the SMS");
      }
      break;

    default:
      break;
  }
}

// Shows on the dashboard whether the SIM800L is answering, before any crash happens.
void pollGSMPresence() {
  if (gsmReady || smsBusy()) {
    return;
  }

  if (gsmBuffer.indexOf("OK") >= 0) {
    gsmReady = true;
    gsmBuffer = "";
    return;
  }

  const uint32_t now = millis();

  if (now - lastGsmProbe >= GSM_PROBE_INTERVAL_MS) {
    lastGsmProbe = now;
    gsmBuffer = "";
    gsmSerial.print("AT\r");
  }
}

// ---------- Accident detection ----------

void triggerAlert(float impactG) {
  alertActive = true;
  alertImpactG = impactG;
  alertStartMs = millis();

  smsBody = "ACCIDENT ALERT! ";
  smsBody += RIDER_NAME;
  smsBody += " may have crashed. Impact ";
  smsBody += String(impactG, 1);
  smsBody += " g. ";

  if (gps.location.isValid()) {
    smsBody += "Location: https://maps.google.com/?q=";
    smsBody += String(gps.location.lat(), 6);
    smsBody += ",";
    smsBody += String(gps.location.lng(), 6);
  } else {
    smsBody += "Location: no GPS fix.";
  }

  startSMS();
}

void resetAlert() {
  alertActive = false;
  alertImpactG = 0.0f;
  digitalWrite(BUZZER_PIN, LOW);
}

// Roll and pitch from a complementary filter. Near upside down (±180°) roll wraps around.
void updateOrientation(
  const sensors_event_t &acceleration,
  const sensors_event_t &angularVelocity,
  float dt
) {
  const float ax = acceleration.acceleration.x;
  const float ay = acceleration.acceleration.y;
  const float az = acceleration.acceleration.z;

  const float accelRoll = atan2f(ay, az) * RAD_TO_DEG;
  const float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  if (!orientationReady) {
    rollDeg = accelRoll;
    pitchDeg = accelPitch;
    orientationReady = true;
    return;
  }

  rollDeg =
    ORIENTATION_GYRO_WEIGHT * (rollDeg + angularVelocity.gyro.x * RAD_TO_DEG * dt) +
    (1.0f - ORIENTATION_GYRO_WEIGHT) * accelRoll;

  pitchDeg =
    ORIENTATION_GYRO_WEIGHT * (pitchDeg + angularVelocity.gyro.y * RAD_TO_DEG * dt) +
    (1.0f - ORIENTATION_GYRO_WEIGHT) * accelPitch;
}

void sampleMPU() {
  if (!mpuDetected) {
    return;
  }

  const uint32_t now = millis();

  if (now - lastMpuSample < MPU_SAMPLE_MS) {
    return;
  }

  const float dt = (lastMpuSample == 0 ? MPU_SAMPLE_MS : now - lastMpuSample) / 1000.0f;
  lastMpuSample = now;

  sensors_event_t acceleration;
  sensors_event_t angularVelocity;
  sensors_event_t temperature;

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

  if (!mpuValid) {
    return;
  }

  accelEvent = acceleration;
  gyroEvent = angularVelocity;
  tempEvent = temperature;

  currentG = sqrtf(
    acceleration.acceleration.x * acceleration.acceleration.x +
    acceleration.acceleration.y * acceleration.acceleration.y +
    acceleration.acceleration.z * acceleration.acceleration.z
  ) / SENSORS_GRAVITY_STANDARD;

  updateOrientation(acceleration, angularVelocity, dt);

  if (currentG >= CRASH_THRESHOLD_G) {
    if (!alertActive) {
      triggerAlert(currentG);
    } else if (currentG > alertImpactG) {
      alertImpactG = currentG;
    }
  }
}

bool buzzerSounding() {
  return alertActive && millis() - alertStartMs < BUZZER_DURATION_MS;
}

void updateBuzzer() {
  const bool on =
    buzzerSounding() &&
    ((millis() - alertStartMs) / BUZZER_BEEP_MS) % 2 == 0;

  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

// ---------- JSON number helper ----------

void appendNumber(String &json, float value, bool valid) {
  if (valid && isfinite(value)) {
    json += String(value, 3);
  } else {
    json += "null";
  }
}

// ---------- Combined GPS + MPU6050 + alert API ----------

void sendGPSData() {
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

  const bool speedValid = gps.speed.isValid();

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
  json.reserve(1200);

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

  json += ",\"speed_kmph\":";
  appendNumber(json, gps.speed.kmph(), speedValid);

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
  appendNumber(json, accelEvent.acceleration.x, mpuValid);

  json += ",\"ay\":";
  appendNumber(json, accelEvent.acceleration.y, mpuValid);

  json += ",\"az\":";
  appendNumber(json, accelEvent.acceleration.z, mpuValid);

  json += ",\"gx\":";
  appendNumber(json, gyroEvent.gyro.x, mpuValid);

  json += ",\"gy\":";
  appendNumber(json, gyroEvent.gyro.y, mpuValid);

  json += ",\"gz\":";
  appendNumber(json, gyroEvent.gyro.z, mpuValid);

  json += ",\"temperature\":";
  appendNumber(json, tempEvent.temperature, mpuValid);

  json += ",\"g\":";
  appendNumber(json, currentG, mpuValid);

  json += ",\"roll\":";
  appendNumber(json, rollDeg, mpuValid && orientationReady);

  json += ",\"pitch\":";
  appendNumber(json, pitchDeg, mpuValid && orientationReady);

  json += "}";

  json += ",\"alert\":{\"active\":";
  json += alertActive ? "true" : "false";

  json += ",\"threshold_g\":";
  json += String(CRASH_THRESHOLD_G, 2);

  json += ",\"impact_g\":";
  appendNumber(json, alertImpactG, alertActive);

  json += ",\"seconds_ago\":";
  if (alertActive) {
    json += String((now - alertStartMs) / 1000);
  } else {
    json += "null";
  }

  json += ",\"buzzer\":";
  json += buzzerSounding() ? "true" : "false";

  json += ",\"sms\":\"";
  json += smsStateName();
  json += "\"";

  json += ",\"sms_detail\":\"";
  json += smsDetail;
  json += "\"";

  json += ",\"gsm_ready\":";
  json += gsmReady ? "true" : "false";

  json += ",\"contact\":\"";
  json += EMERGENCY_NUMBER;
  json += "\"";

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
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  gpsSerial.setRxBufferSize(2048);

  gpsSerial.begin(
    GPS_BAUD,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  gsmSerial.begin(
    GSM_BAUD,
    SERIAL_8N1,
    GSM_RX_PIN,
    GSM_TX_PIN
  );

  gsmBuffer.reserve(256);
  smsBody.reserve(200);

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
    // Copy the page into RAM so the Maps API key can be inserted.
    String page = FPSTR(INDEX_HTML);
    page.replace("%MAPS_KEY%", GOOGLE_MAPS_API_KEY);

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/html; charset=utf-8", page);
  });

  server.on("/api/gps", HTTP_GET, sendGPSData);

  server.on("/api/alert/reset", HTTP_POST, []() {
    resetAlert();
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.onNotFound([]() {
    server.sendHeader("Cache-Control", "no-store");
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
}

// ---------- Main loop ----------

void loop() {
  readGPS();
  sampleMPU();
  updateBuzzer();
  updateSMS();
  pollGSMPresence();
  server.handleClient();
  readGPS();
  delay(1);
}
