#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
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
const char *ROUTER_SSID = "SYIMAFAMY_2.4GHz@unifi";
const char *ROUTER_PASSWORD = "SYIMA3108";

// The dashboard also answers at http://riderdashboard.local on either network.
const char *MDNS_NAME = "riderdashboard";

IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

// ---------- Rider and emergency contact ----------

const char *RIDER_NAME = "Muhammad Irfan Bin Yusof";
const char *VEHICLE_MODEL = "Honda EX5";
const char *VEHICLE_PLATE = "JBR 1240";

// International format. Replace before use: no SMS is sent while it still contains 'X'.
const char *EMERGENCY_NUMBER = "+60XXXXXXXXX";

// ---------- Map ----------

// The dashboard map uses Leaflet with free OpenStreetMap tiles: no API key or billing needed.
// The viewing PC or phone needs Internet access for the map; everything else works offline.

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
String smsDetail;

// A crash sends two texts: rider and motorcycle details, then the location.
// Each stays under the 160-character limit of a single SMS.
constexpr int SMS_MESSAGE_COUNT = 2;
String smsMessages[SMS_MESSAGE_COUNT];
int smsIndex = 0;

// ---------- Embedded website ----------

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="theme-color" content="#e4e9fb">
<title>Rider Tracker</title>
<!-- Loaded without blocking the page: offline it falls back to the system fonts. -->
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" media="print" onload="this.media='all'"
      href="https://fonts.googleapis.com/css2?family=Inter:opsz,wght@14..32,400..800&display=swap">
<style>
  /* Liquid Glass theme, inspired by Apple's iOS 26 / macOS Tahoe look:
     translucent frosted panels floating over a soft colourful wallpaper. */
  :root {
    --brand: #007aff;
    --brand-dark: #0062d1;
    --brand-soft: rgba(0, 122, 255, 0.13);
    --coral: #ff9500;
    --coral-dark: #c96f00;
    --coral-soft: rgba(255, 149, 0, 0.15);
    --ink: #1d1d1f;
    --muted: #6e6e73;
    --line: rgba(60, 60, 67, 0.12);
    --surface: rgba(255, 255, 255, 0.5);
    --fill: rgba(255, 255, 255, 0.46);
    --fill-strong: rgba(255, 255, 255, 0.78);
    --track: rgba(120, 120, 128, 0.16);
    --green: #1f9d47;
    --green-soft: rgba(52, 199, 89, 0.16);
    --red: #e5251b;
    --red-soft: rgba(255, 59, 48, 0.13);
    --amber: #c46a00;
    --amber-soft: rgba(255, 159, 10, 0.16);
    --blue: #0066d6;
    --blue-soft: rgba(0, 122, 255, 0.12);

    /* The glass material shared by every floating panel. */
    --glass-bg: linear-gradient(160deg, rgba(255, 255, 255, 0.66) 0%, rgba(255, 255, 255, 0.36) 100%);
    --glass-edge: rgba(255, 255, 255, 0.75);
    --glass-blur: blur(26px) saturate(190%);
    --glass-shadow:
      inset 0 1px 0 rgba(255, 255, 255, 0.95),
      inset 0 -1px 1px rgba(255, 255, 255, 0.35),
      0 12px 32px rgba(38, 48, 90, 0.12),
      0 1px 3px rgba(38, 48, 90, 0.08);
    --glass-shadow-sm:
      inset 0 1px 0 rgba(255, 255, 255, 0.95),
      0 4px 14px rgba(38, 48, 90, 0.14);

    /* iOS uses San Francisco everywhere, numbers included. Apple devices get the real SF Pro
       from the system; anywhere else Inter (the closest free match) stands in. */
    --sans: -apple-system, BlinkMacSystemFont, "SF Pro Text", "SF Pro Display", "SF Pro", "Inter", "Segoe UI Variable Text", system-ui, sans-serif;
    --mono: "SF Mono", ui-monospace, Menlo, "Cascadia Mono", Consolas, monospace;
    color-scheme: light;
  }

  * { box-sizing: border-box; }

  html, body { height: 100%; margin: 0; }

  body {
    display: flex;
    gap: 12px;
    padding: 12px;
    color: var(--ink);
    font: 14px/1.45 var(--sans);
    letter-spacing: -0.01em;
    font-optical-sizing: auto;
    font-variant-numeric: tabular-nums;
    -webkit-font-smoothing: antialiased;
    background-color: #e4e9fb;
    background-image:
      radial-gradient(900px 700px at 0% 0%, rgba(90, 200, 250, 0.55), transparent 60%),
      radial-gradient(800px 640px at 100% 0%, rgba(255, 140, 190, 0.45), transparent 60%),
      radial-gradient(900px 760px at 100% 100%, rgba(255, 190, 110, 0.5), transparent 60%),
      radial-gradient(860px 720px at 0% 100%, rgba(120, 220, 170, 0.5), transparent 60%),
      linear-gradient(135deg, #dce6ff 0%, #efe4ff 50%, #ffe9e0 100%);
    background-attachment: fixed;
  }

  button, input { font: inherit; }

  [hidden] { display: none !important; }

  /* ---------- Sidebar ---------- */

  .sidebar {
    flex: none;
    display: flex;
    flex-direction: column;
    width: 236px;
    padding: 20px 12px;
    border: 1px solid var(--glass-edge);
    border-radius: 28px;
    background: var(--glass-bg);
    -webkit-backdrop-filter: var(--glass-blur);
    backdrop-filter: var(--glass-blur);
    box-shadow: var(--glass-shadow);
  }

  .logo {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 0 8px 24px;
  }

  .logo-mark {
    display: grid;
    place-items: center;
    width: 40px;
    height: 40px;
    border-radius: 12px;
    background: linear-gradient(160deg, #5ac8fa 0%, #007aff 60%, #5856d6 100%);
    color: #fff;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.55), 0 6px 14px rgba(0, 122, 255, 0.35);
  }

  .logo-mark svg { width: 22px; height: 22px; }
  .logo-name { font-weight: 700; font-size: 1.02rem; line-height: 1.15; letter-spacing: -0.02em; }
  .logo-sub { font-size: 0.72rem; color: var(--muted); }

  .nav {
    display: flex;
    flex-direction: column;
    gap: 4px;
  }

  .nav a {
    position: relative;
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 6px 12px 6px 6px;
    border: 1px solid transparent;
    border-radius: 999px;
    color: var(--ink);
    font-weight: 500;
    text-decoration: none;
    transition: background-color 0.2s ease, box-shadow 0.2s ease, color 0.2s ease;
  }

  /* White glyph on a coloured rounded square, like the iOS Settings list. */
  .nav-icon {
    --tile: #8e8e93;
    --tile-top: #aeaeb2;
    --knock: var(--tile);
    flex: none;
    display: grid;
    place-items: center;
    width: 30px;
    height: 30px;
    border-radius: 8px;
    background: linear-gradient(180deg, var(--tile-top), var(--tile));
    color: #fff;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.35), 0 1px 3px rgba(0, 0, 0, 0.14);
  }

  .nav-icon svg { width: 19px; height: 19px; }
  .nav-icon.blue   { --tile: #007aff; --tile-top: #3d9bff; }
  .nav-icon.purple { --tile: #5856d6; --tile-top: #7d7bea; }
  .nav-icon.red    { --tile: #ff3b30; --tile-top: #ff6961; }
  .nav-icon.orange { --tile: #ff9500; --tile-top: #ffb340; }
  .nav-icon.green  { --tile: #34c759; --tile-top: #5fd97c; }

  /* Parts of a filled glyph that are "cut out" in SF Symbols take the colour behind the icon. */
  .knock { stroke: var(--knock, #fff); fill: none; }
  .knock-fill { fill: var(--knock, #fff); }
  svg.tint { color: var(--brand); }
  .nav a:hover { background: rgba(255, 255, 255, 0.4); }

  /* The selected page sits in a brighter glass bubble, like a macOS sidebar selection. */
  .nav a[aria-current="page"] {
    border-color: rgba(255, 255, 255, 0.9);
    background: linear-gradient(180deg, rgba(255, 255, 255, 0.92), rgba(255, 255, 255, 0.66));
    color: var(--brand);
    font-weight: 600;
    box-shadow: var(--glass-shadow-sm);
  }

  .nav-badge {
    margin-left: auto;
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: #ff3b30;
    box-shadow: 0 0 0 2px rgba(255, 255, 255, 0.9);
    animation: flash 1s infinite;
  }

  .user-pill {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-top: auto;
    padding: 6px 14px 6px 6px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    border-radius: 999px;
    background: var(--fill);
    font-weight: 600;
    font-size: 0.88rem;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.9);
  }

  .avatar-sm {
    flex: none;
    display: grid;
    place-items: center;
    width: 32px;
    height: 32px;
    border-radius: 50%;
    background: linear-gradient(160deg, #34c759, #0a9b8a);
    color: #fff;
    font-size: 0.75rem;
    font-weight: 700;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.5);
  }

  /* ---------- Main area ---------- */

  .main {
    flex: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
    gap: 12px;
  }

  .page-header {
    flex: none;
    display: flex;
    align-items: center;
    gap: 16px;
    padding: 14px 22px;
    border: 1px solid var(--glass-edge);
    border-radius: 24px;
    background: var(--glass-bg);
    -webkit-backdrop-filter: var(--glass-blur);
    backdrop-filter: var(--glass-blur);
    box-shadow: var(--glass-shadow);
  }

  .page-header h1 {
    margin: 0;
    font-size: 1.75rem;
    font-weight: 700;
    letter-spacing: -0.025em;
  }

  .header-right {
    display: flex;
    align-items: center;
    gap: 14px;
    margin-left: auto;
    text-align: right;
  }

  .updated {
    font-size: 0.82rem;
    color: var(--muted);
    font-variant-numeric: tabular-nums;
  }

  .updated strong { display: block; color: var(--ink); font-weight: 600; }

  .pill {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    padding: 6px 13px;
    border-radius: 999px;
    border: 1px solid rgba(255, 255, 255, 0.8);
    background: var(--fill-strong);
    font-size: 0.8rem;
    font-weight: 600;
    white-space: nowrap;
    box-shadow: var(--glass-shadow-sm);
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
  .live-pill.on::before { box-shadow: 0 0 0 3px rgba(52, 199, 89, 0.25); animation: pulse 1.6s ease-in-out infinite; }

  .crash-banner {
    flex: none;
    display: flex;
    align-items: center;
    gap: 16px;
    padding: 12px 14px 12px 22px;
    border: 1px solid rgba(255, 255, 255, 0.45);
    border-radius: 22px;
    --knock: #e5251b;
    background: linear-gradient(160deg, rgba(255, 69, 58, 0.88), rgba(215, 0, 21, 0.8));
    -webkit-backdrop-filter: var(--glass-blur);
    backdrop-filter: var(--glass-blur);
    color: #fff;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.45), 0 12px 30px rgba(215, 0, 21, 0.3);
  }

  .crash-banner svg { flex: none; width: 26px; height: 26px; animation: flash 1s infinite; }
  .crash-banner strong { font-size: 1rem; }
  .crash-banner span { opacity: 0.92; }

  .crash-banner .btn {
    width: auto;
    margin: 0 0 0 auto;
    padding: 0 20px;
    min-height: 38px;
    background: rgba(255, 255, 255, 0.92);
    color: #d70015;
    box-shadow: inset 0 1px 0 #fff, 0 4px 12px rgba(120, 0, 10, 0.25);
  }

  .content {
    flex: 1;
    min-height: 0;
    overflow-y: auto;
    padding: 2px 4px 8px;
    margin: -2px -4px 0;
  }

  /* ---------- Shared pieces ---------- */

  .card {
    padding: 20px;
    border: 1px solid var(--glass-edge);
    border-radius: 24px;
    background: var(--glass-bg);
    -webkit-backdrop-filter: var(--glass-blur);
    backdrop-filter: var(--glass-blur);
    box-shadow: var(--glass-shadow);
  }

  .card-title {
    display: flex;
    align-items: center;
    gap: 8px;
    margin: 0 0 16px;
    font-size: 1.0625rem;
    font-weight: 600;
    letter-spacing: -0.025em;
  }

  .card-actions {
    display: flex;
    gap: 8px;
    margin-left: auto;
  }

  .tag {
    padding: 3px 10px;
    border-radius: 999px;
    background: var(--brand-soft);
    color: var(--brand-dark);
    font-size: 0.72rem;
    font-weight: 600;
  }

  .tag.right { margin-left: auto; }

  .sub-title {
    margin: 20px 0 10px;
    font-size: 0.9rem;
    font-weight: 600;
  }

  .label {
    display: block;
    font-size: 0.7rem;
    font-weight: 600;
    letter-spacing: 0.04em;
    text-transform: uppercase;
    color: var(--muted);
  }

  .value {
    margin: 2px 0;
    font: 600 1.05rem/1.3 var(--sans);
    font-variant-numeric: tabular-nums;
    overflow-wrap: anywhere;
  }

  .value.stale { color: var(--amber); }

  .note {
    font-size: 0.75rem;
    font-weight: 400;
    color: var(--muted);
  }

  .helper-text {
    margin: 10px 0 0;
    font-size: 0.8rem;
    color: var(--muted);
  }

  .grid2 {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 10px;
  }

  .tile {
    padding: 12px 14px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    border-radius: 16px;
    background: var(--fill);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.85);
  }

  .status {
    --tone: var(--blue);
    --tone-bg: var(--blue-soft);
    display: flex;
    align-items: center;
    gap: 10px;
    margin: 12px 0;
    padding: 9px 14px;
    border: 1px solid rgba(255, 255, 255, 0.6);
    border-radius: 14px;
    background: var(--tone-bg);
    color: var(--tone);
    font-size: 0.85rem;
    font-weight: 600;
  }

  .status::before {
    content: "";
    flex: none;
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background: var(--tone);
  }

  .status.info  { --tone: var(--blue);  --tone-bg: var(--blue-soft); }
  .status.ok    { --tone: var(--green); --tone-bg: var(--green-soft); }
  .status.warn  { --tone: var(--amber); --tone-bg: var(--amber-soft); }
  .status.error { --tone: var(--red);   --tone-bg: var(--red-soft); }

  /* Capsule buttons with a glossy top edge, like iOS filled buttons. */
  .btn {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    width: 100%;
    min-height: 44px;
    margin: 12px 0;
    border: 0;
    border-radius: 999px;
    background: linear-gradient(180deg, #3d9bff 0%, #007aff 100%);
    color: #fff;
    font-weight: 600;
    text-decoration: none;
    cursor: pointer;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.45), 0 6px 16px rgba(0, 122, 255, 0.3);
    transition: filter 0.15s ease, transform 0.15s ease;
  }

  .btn svg { width: 18px; height: 18px; }
  .btn:hover { filter: brightness(1.06); }
  .btn:active { transform: scale(0.98); }
  .btn.coral {
    background: linear-gradient(180deg, #ffad33 0%, #ff9500 100%);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.45), 0 6px 16px rgba(255, 149, 0, 0.3);
  }

  .btn[aria-disabled="true"],
  .btn:disabled {
    background: rgba(255, 255, 255, 0.45);
    color: #a1a1a6;
    box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.7);
    pointer-events: none;
  }

  .btn-small {
    display: inline-flex;
    align-items: center;
    gap: 6px;
    padding: 6px 13px;
    border: 1px solid rgba(255, 255, 255, 0.8);
    border-radius: 999px;
    background: var(--fill-strong);
    -webkit-backdrop-filter: blur(14px) saturate(180%);
    backdrop-filter: blur(14px) saturate(180%);
    color: var(--ink);
    font-size: 0.8rem;
    font-weight: 500;
    cursor: pointer;
    box-shadow: var(--glass-shadow-sm);
    transition: background-color 0.15s ease;
  }

  .btn-small:hover { background: rgba(255, 255, 255, 0.95); }
  .btn-small:disabled { color: #a1a1a6; cursor: default; background: var(--fill); }
  .btn-small svg { width: 15px; height: 15px; }

  .url-row {
    display: flex;
    gap: 8px;
    margin-top: 6px;
  }

  .url-field {
    flex: 1;
    min-width: 0;
    padding: 8px 14px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    border-radius: 999px;
    background: var(--track);
    color: var(--ink);
    font: 0.78rem/1.4 var(--mono);
  }

  .list {
    border: 1px solid rgba(255, 255, 255, 0.7);
    border-radius: 16px;
    background: var(--fill);
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
  .row strong { text-align: right; font-weight: 600; overflow-wrap: anywhere; }

  /* ---------- Map ---------- */

  .map {
    position: relative;
    min-height: 320px;
    background-color: rgba(235, 240, 250, 0.6);
    background-image:
      linear-gradient(rgba(0, 122, 255, 0.07) 1px, transparent 1px),
      linear-gradient(90deg, rgba(0, 122, 255, 0.07) 1px, transparent 1px);
    background-size: 32px 32px;
  }

  .leaflet-map {
    position: absolute;
    inset: 0;
    z-index: 0;
    background: transparent;
    font-family: var(--sans);
  }

  .leaflet-map .leaflet-control-attribution {
    margin: 0 8px 8px 0;
    padding: 2px 8px;
    border-radius: 999px;
    background: rgba(255, 255, 255, 0.7);
    -webkit-backdrop-filter: blur(12px);
    backdrop-filter: blur(12px);
  }

  /* Rider pin on the Leaflet map: a teardrop with the rider's initials. */
  .rider-pin { background: none; border: 0; }

  .rider-pin span {
    display: grid;
    place-items: center;
    width: 40px;
    height: 40px;
    border: 3px solid #fff;
    border-radius: 50% 50% 50% 4px;
    background: linear-gradient(135deg, #3d9bff, #007aff);
    box-shadow: 0 6px 16px rgba(0, 60, 140, 0.35);
    transform: rotate(-45deg);
  }

  .rider-pin b {
    color: #fff;
    font: 700 12px/1 var(--sans);
    transform: rotate(45deg);
  }

  .rider-pin.crash span { background: #ff3b30; animation: flash 1s infinite; }

  .map-empty {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    padding: 24px;
  }

  .map-empty span {
    max-width: 440px;
    padding: 10px 18px;
    border: 1px solid rgba(255, 255, 255, 0.8);
    border-radius: 999px;
    background: var(--fill-strong);
    -webkit-backdrop-filter: var(--glass-blur);
    backdrop-filter: var(--glass-blur);
    color: var(--muted);
    font-weight: 600;
    text-align: center;
    box-shadow: var(--glass-shadow-sm);
  }

  .map-overlay {
    position: absolute;
    z-index: 1;
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .map-overlay.top { top: 14px; left: 14px; right: 14px; }
  .map-overlay.bottom { right: 14px; bottom: 34px; flex-direction: column; gap: 0; }

  .map-chip {
    padding: 6px 13px;
    border: 1px solid rgba(255, 255, 255, 0.8);
    border-radius: 999px;
    background: var(--fill-strong);
    -webkit-backdrop-filter: blur(14px) saturate(180%);
    backdrop-filter: blur(14px) saturate(180%);
    font-size: 0.78rem;
    font-weight: 600;
    color: var(--muted);
    box-shadow: var(--glass-shadow-sm);
    font-variant-numeric: tabular-nums;
  }

  .map-overlay.top .btn-small { margin-left: auto; }

  /* Zoom buttons and level read as one glass capsule. */
  .zoom-btn,
  .zoom-level {
    display: grid;
    place-items: center;
    width: 38px;
    border: 1px solid rgba(255, 255, 255, 0.8);
    background: var(--fill-strong);
    -webkit-backdrop-filter: blur(14px) saturate(180%);
    backdrop-filter: blur(14px) saturate(180%);
    color: var(--ink);
  }

  .zoom-btn {
    height: 38px;
    font-size: 1.15rem;
    cursor: pointer;
    box-shadow: var(--glass-shadow-sm);
  }

  .zoom-btn:first-child { border-radius: 19px 19px 0 0; border-bottom: 0; }
  .zoom-btn:last-child { border-radius: 0 0 19px 19px; border-top: 0; }
  .zoom-btn:hover { background: rgba(255, 255, 255, 0.95); }

  .zoom-level {
    height: 24px;
    border-top: 0;
    border-bottom: 0;
    font: 600 0.72rem/1 var(--sans);
    color: var(--muted);
  }

  /* ---------- Split layout (Home, Safety) ---------- */

  .split {
    display: grid;
    grid-template-columns: 360px minmax(0, 1fr);
    height: calc(100vh - 128px);
    min-height: 540px;
    overflow: hidden;
    border: 1px solid var(--glass-edge);
    border-radius: 26px;
    background: var(--glass-bg);
    -webkit-backdrop-filter: var(--glass-blur);
    backdrop-filter: var(--glass-blur);
    box-shadow: var(--glass-shadow);
  }

  .split-panel {
    min-height: 0;
    overflow-y: auto;
    padding: 20px;
    border-right: 1px solid rgba(255, 255, 255, 0.6);
  }

  .split .map { min-height: 0; height: 100%; }

  .panel-title {
    display: flex;
    align-items: center;
    gap: 8px;
    margin: 0 0 14px;
    font-size: 1.05rem;
    font-weight: 600;
    letter-spacing: -0.015em;
  }

  /* Rider list item, like a fleet list row */
  .rider-item {
    display: flex;
    gap: 12px;
    padding: 14px;
    border: 1px solid rgba(255, 255, 255, 0.75);
    border-radius: 18px;
    background: var(--fill);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.85);
  }

  .rider-num {
    flex: none;
    display: grid;
    place-items: center;
    width: 26px;
    height: 26px;
    border-radius: 50%;
    background: #aeaeb2;
    color: #fff;
    font-size: 0.72rem;
    font-weight: 700;
  }

  .rider-num.live { background: var(--brand); }
  .rider-num.crash { background: #ff3b30; animation: flash 1s infinite; }

  .rider-info { min-width: 0; flex: 1; }
  .rider-name { font-weight: 600; }

  .rider-line {
    display: flex;
    align-items: center;
    gap: 6px;
    margin-top: 3px;
    font-size: 0.8rem;
    color: var(--muted);
  }

  .rider-line svg { flex: none; width: 14px; height: 14px; }

  .rider-status {
    align-self: flex-start;
    padding: 3px 10px;
    border-radius: 999px;
    background: var(--track);
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
  }

  .rider-status.live { background: var(--brand-soft); color: var(--brand-dark); }
  .rider-status.crash { background: #ff3b30; color: #fff; }

  /* ---------- Analytics ---------- */

  .kpis {
    display: grid;
    grid-template-columns: repeat(4, minmax(0, 1fr));
    gap: 14px;
    margin-bottom: 14px;
  }

  .kpi-label { font-weight: 500; font-size: 0.88rem; color: var(--muted); }

  .kpi-value {
    margin-top: 8px;
    font-size: 1.8rem;
    font-weight: 700;
    letter-spacing: -0.03em;
    font-variant-numeric: tabular-nums;
  }

  .kpi-value small { margin-left: 4px; font-size: 0.9rem; font-weight: 600; color: var(--muted); letter-spacing: 0; }

  .charts {
    display: grid;
    grid-template-columns: minmax(0, 1.25fr) minmax(0, 1fr);
    gap: 14px;
    margin-bottom: 14px;
  }

  .chart {
    display: block;
    width: 100%;
    height: 240px;
  }

  .legend {
    display: flex;
    flex-wrap: wrap;
    gap: 16px;
    margin-top: 10px;
    font-size: 0.75rem;
    color: var(--muted);
  }

  .legend span { display: inline-flex; align-items: center; gap: 6px; }

  .legend span::before {
    content: "";
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--swatch);
  }

  .table {
    width: 100%;
    border-collapse: collapse;
  }

  .table th {
    padding: 10px 12px;
    font-size: 0.75rem;
    font-weight: 500;
    color: var(--muted);
    text-align: left;
    border-bottom: 1px solid var(--line);
  }

  .table td {
    padding: 14px 12px;
    font-variant-numeric: tabular-nums;
    border-bottom: 1px solid var(--line);
  }

  .table tr:last-child td { border-bottom: 0; }
  .table td.num { font-family: var(--sans); font-weight: 600; }
  .table td:first-child { font-weight: 600; }

  /* ---------- Safety ---------- */

  .safety-hero {
    display: flex;
    align-items: center;
    gap: 14px;
    padding: 14px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    border-radius: 18px;
    background: var(--green-soft);
  }

  .safety-icon {
    flex: none;
    display: grid;
    place-items: center;
    width: 44px;
    height: 44px;
    border-radius: 50%;
    background: linear-gradient(160deg, #4cd964, #28a745);
    color: #fff;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.5), 0 4px 12px rgba(40, 167, 69, 0.3);
  }

  .safety-icon svg { width: 24px; height: 24px; }
  .safety-title { font-size: 1.05rem; font-weight: 600; color: var(--green); }
  .safety-sub { font-size: 0.82rem; color: var(--muted); }

  .safety-hero.alert { background: var(--red-soft); }
  .safety-hero.alert .safety-icon {
    background: linear-gradient(160deg, #ff6961, #e5251b);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.5), 0 4px 12px rgba(229, 37, 27, 0.35);
    animation: flash 1s infinite;
  }
  .safety-hero.alert .safety-title { color: var(--red); }

  .meter-head {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
    margin-top: 18px;
  }

  .meter-head strong {
    font: 700 1.25rem/1 var(--sans);
    font-variant-numeric: tabular-nums;
  }

  .meter {
    position: relative;
    height: 10px;
    margin: 10px 0 4px;
    border-radius: 999px;
    background: var(--track);
    overflow: hidden;
  }

  .meter-fill {
    position: absolute;
    top: 0;
    bottom: 0;
    left: 0;
    width: 0;
    border-radius: 999px;
    background: var(--green);
    transition: width 0.3s ease-out, background-color 0.3s;
  }

  .meter-mark {
    position: absolute;
    top: 0;
    bottom: 0;
    left: 62.5%;
    width: 3px;
    border-radius: 2px;
    background: var(--ink);
  }

  .meter-scale {
    position: relative;
    height: 1.4em;
    font-size: 0.72rem;
    color: var(--muted);
  }

  .meter-scale span { position: absolute; }
  .meter-scale span:last-child { right: 0; }

  #thresholdLabel {
    left: 62.5%;
    transform: translateX(-50%);
    font-weight: 600;
    color: var(--ink);
  }

  /* Read-only "alert settings" rows, styled like the geofence settings list */
  .setting {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
    padding: 10px 0;
    border-top: 1px solid var(--line);
    font-size: 0.85rem;
  }

  .setting:first-child { border-top: 0; }

  .state {
    flex: none;
    padding: 3px 10px;
    border-radius: 999px;
    background: var(--track);
    color: var(--muted);
    font-size: 0.72rem;
    font-weight: 600;
  }

  .state.on { background: var(--coral-soft); color: var(--coral-dark); }
  .state.ok { background: var(--green-soft); color: var(--green); }
  .state.bad { background: var(--red-soft); color: var(--red); }

  /* ---------- Vehicle ---------- */

  .vehicle-layout {
    display: grid;
    grid-template-columns: minmax(0, 1.5fr) minmax(320px, 1fr);
    gap: 14px;
    align-items: start;
  }

  .orient3d {
    position: relative;
    overflow: hidden;
    border: 1px solid rgba(255, 255, 255, 0.75);
    border-radius: 20px;
    background:
      radial-gradient(120% 80% at 50% 0%, rgba(255, 255, 255, 0.75), transparent 70%),
      linear-gradient(180deg, rgba(225, 236, 255, 0.55) 0%, rgba(205, 222, 250, 0.45) 100%);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.9);
  }

  .orient3d canvas {
    display: block;
    width: 100%;
    height: 420px;
    cursor: grab;
    touch-action: none;
  }

  .orient3d canvas.dragging { cursor: grabbing; }

  /* 3D / Rear / Side switch as an Apple segmented control. */
  .view-buttons {
    position: absolute;
    top: 12px;
    left: 12px;
    display: flex;
    gap: 2px;
    padding: 3px;
    border: 1px solid rgba(255, 255, 255, 0.8);
    border-radius: 999px;
    background: rgba(255, 255, 255, 0.45);
    -webkit-backdrop-filter: blur(14px) saturate(180%);
    backdrop-filter: blur(14px) saturate(180%);
    box-shadow: var(--glass-shadow-sm);
  }

  .view-buttons .btn-small {
    border-color: transparent;
    background: transparent;
    box-shadow: none;
    -webkit-backdrop-filter: none;
    backdrop-filter: none;
  }

  .view-buttons .btn-small[aria-pressed="true"] {
    border-color: rgba(255, 255, 255, 0.95);
    background: #fff;
    color: var(--brand);
    font-weight: 600;
    box-shadow: 0 2px 8px rgba(38, 48, 90, 0.16);
  }

  .orient-hint {
    position: absolute;
    right: 14px;
    bottom: 10px;
    font-size: 0.75rem;
    color: var(--muted);
    pointer-events: none;
  }

  .attitude {
    padding: 14px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    border-radius: 18px;
    background: var(--brand-soft);
    text-align: center;
  }

  .attitude-value {
    font: 700 1.7rem/1.1 var(--sans);
    font-variant-numeric: tabular-nums;
    color: var(--brand-dark);
  }

  .attitude-dir { font-size: 0.8rem; font-weight: 600; color: var(--muted); }

  .axis-name {
    display: inline-flex;
    align-items: center;
    gap: 8px;
  }

  .axis-name::before {
    content: "";
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--axis);
  }

  .axis-value {
    font: 600 0.95rem/1.2 var(--sans);
    font-variant-numeric: tabular-nums;
  }

  .axis-x { --axis: #ff3b30; }
  .axis-y { --axis: #34c759; }
  .axis-z { --axis: #007aff; }

  /* ---------- Rider profile ---------- */

  .profile {
    display: grid;
    grid-template-columns: 200px minmax(0, 1fr) 220px;
    gap: 28px;
    align-items: start;
  }

  .profile-photo {
    display: grid;
    place-items: center;
    aspect-ratio: 1;
    border-radius: 28px;
    background: linear-gradient(160deg, #5ac8fa 0%, #007aff 55%, #5856d6 100%);
    color: #fff;
    font-size: 3.4rem;
    font-weight: 700;
    letter-spacing: -0.02em;
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.5), 0 14px 30px rgba(0, 90, 200, 0.3);
  }

  .profile-name { margin: 0; font-size: 1.6rem; font-weight: 700; letter-spacing: -0.025em; }

  .profile-ids {
    display: flex;
    flex-wrap: wrap;
    gap: 16px;
    margin: 6px 0 16px;
    font-size: 0.85rem;
    color: var(--muted);
  }

  .profile-ids span { display: inline-flex; align-items: center; gap: 6px; }
  .profile-ids svg { width: 15px; height: 15px; }

  .info-row {
    display: grid;
    grid-template-columns: 170px minmax(0, 1fr);
    gap: 12px;
    padding: 9px 0;
    border-bottom: 1px solid var(--line);
    font-size: 0.88rem;
  }

  .info-row span:first-child { color: var(--muted); }
  .info-row span:last-child { font-weight: 500; overflow-wrap: anywhere; }

  .stat-chips {
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  .stat-chip {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 12px 14px;
    border: 1px solid rgba(255, 255, 255, 0.7);
    border-radius: 18px;
    background: var(--fill);
    box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.85);
  }

  .stat-chip svg { flex: none; width: 20px; height: 20px; color: var(--muted); }
  .stat-chip.coral { background: var(--coral-soft); }
  .stat-chip.coral svg { color: var(--coral-dark); }
  .stat-chip.green { background: var(--green-soft); }
  .stat-chip.green svg { color: var(--green); }

  .stat-chip strong {
    display: block;
    font: 600 1.05rem/1.2 var(--sans);
    font-variant-numeric: tabular-nums;
  }

  .stat-chip span { font-size: 0.75rem; color: var(--muted); }

  a:focus-visible,
  button:focus-visible,
  input:focus-visible {
    outline: 3px solid rgba(0, 122, 255, 0.55);
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

  @media (max-width: 1200px) {
    .kpis { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .charts, .vehicle-layout { grid-template-columns: 1fr; }
    .profile { grid-template-columns: 160px minmax(0, 1fr); }
    .stat-chips { grid-column: 1 / -1; flex-direction: row; flex-wrap: wrap; }
  }

  /* Narrow windows: the sidebar becomes a floating top bar and pages stack. */
  @media (max-width: 900px) {
    body { display: block; height: auto; padding: 10px; }
    .sidebar { width: auto; flex-direction: row; align-items: center; gap: 12px; margin-bottom: 10px; padding: 8px 10px; border-radius: 999px; overflow-x: auto; scrollbar-width: none; }
    .logo { padding: 0; }
    .logo-text, .user-pill { display: none; }
    .nav { flex-direction: row; }
    .nav a { padding: 8px 12px; white-space: nowrap; }
    .page-header, .crash-banner { flex-wrap: wrap; padding: 12px 16px; }
    .content { padding: 0; margin: 0; overflow: visible; }
    .split { grid-template-columns: 1fr; height: auto; }
    .split-panel { border-right: 0; border-bottom: 1px solid rgba(255, 255, 255, 0.6); }
    .split .map { height: 380px; }
    .profile { grid-template-columns: 1fr; }
    .profile-photo { max-width: 160px; }
  }

  /* Browsers without backdrop blur, or users who ask for less transparency, get solid panels. */
  @supports not ((backdrop-filter: blur(1px)) or (-webkit-backdrop-filter: blur(1px))) {
    :root { --glass-bg: rgba(255, 255, 255, 0.9); }
  }

  @media (prefers-reduced-transparency: reduce) {
    :root { --glass-bg: #fbfbfd; --fill: #f2f2f7; --fill-strong: #fff; --glass-blur: none; }
  }

  @media (prefers-reduced-motion: reduce) {
    * { animation: none !important; transition: none !important; }
  }
</style>
</head>
<body>
  <!-- ---------- Sidebar ---------- -->
  <aside class="sidebar">
    <div class="logo">
      <div class="logo-mark" aria-hidden="true">
        <svg viewBox="0 0 24 24"><path d="M19.6 3.3 3.9 9.9c-.9.4-.8 1.7.2 1.9l6.5 1.5 1.5 6.5c.2 1 1.5 1.1 1.9.2l6.6-15.7c.3-.7-.4-1.4-1-1Z" fill="currentColor"/></svg>
      </div>
      <div class="logo-text">
        <div class="logo-name">Rider Tracker</div>
        <div class="logo-sub">Data Is Sourced By ESP32</div>
      </div>
    </div>

    <nav class="nav" aria-label="Dashboard pages">
      <a href="#home" data-page="home">
        <span class="nav-icon blue" aria-hidden="true"><svg viewBox="0 0 24 24"><path d="M12 3.1c.3 0 .6.1.9.3l7.8 6.6a1 1 0 0 1-1.3 1.5l-.5-.4V19a2 2 0 0 1-2 2h-2.8v-5.2a1 1 0 0 0-1-1h-2.2a1 1 0 0 0-1 1V21H7.1a2 2 0 0 1-2-2v-7.9l-.5.4a1 1 0 0 1-1.3-1.5l7.8-6.6c.3-.2.6-.3.9-.3Z" fill="currentColor"/></svg></span>
        Home
      </a>
      <a href="#analytics" data-page="analytics">
        <span class="nav-icon purple" aria-hidden="true"><svg viewBox="0 0 24 24"><rect x="3.5" y="12.5" width="4.4" height="8" rx="1.3" fill="currentColor"/><rect x="9.8" y="7.5" width="4.4" height="13" rx="1.3" fill="currentColor"/><rect x="16.1" y="3.5" width="4.4" height="17" rx="1.3" fill="currentColor"/></svg></span>
        Analytics
      </a>
      <a href="#safety" data-page="safety">
        <span class="nav-icon red" aria-hidden="true"><svg viewBox="0 0 24 24"><path d="M12 2.5c.2 0 .4 0 .6.1l6.6 2.6c.5.2.8.6.8 1.1v5.4c0 4.7-3.2 8.6-7.6 9.8a1.6 1.6 0 0 1-.8 0C7.2 20.3 4 16.4 4 11.7V6.3c0-.5.3-.9.8-1.1l6.6-2.6c.2-.1.4-.1.6-.1Z" fill="currentColor"/><path class="knock" d="m8.7 12 2.3 2.3 4.4-4.7" fill="none" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"/></svg></span>
        Safety
        <span id="navBadge" class="nav-badge" hidden></span>
      </a>
      <a href="#vehicle" data-page="vehicle">
        <span class="nav-icon orange" aria-hidden="true"><svg viewBox="0 0 24 24"><circle cx="5.8" cy="17.2" r="2.8" fill="none" stroke="currentColor" stroke-width="2.1"/><circle cx="18.4" cy="17.2" r="2.8" fill="none" stroke="currentColor" stroke-width="2.1"/><path d="M2.8 14.6a3.6 3.6 0 0 1 3.6-3.6h3.8c.6 0 1 .4 1 1v3.6H2.8Z" fill="currentColor"/><path d="M5.4 8.8h5.2M10.8 15.6h6.6M16.2 4.8l2.2 12.4M14.6 4.8h3.4" fill="none" stroke="currentColor" stroke-width="2.1" stroke-linecap="round" stroke-linejoin="round"/></svg></span>
        Vehicle
      </a>
      <a href="#rider" data-page="rider">
        <span class="nav-icon green" aria-hidden="true"><svg viewBox="0 0 24 24"><circle cx="12" cy="7.6" r="4.1" fill="currentColor"/><path d="M4.3 19.6c0-3.6 3.4-6.2 7.7-6.2s7.7 2.6 7.7 6.2c0 .8-.6 1.4-1.4 1.4H5.7c-.8 0-1.4-.6-1.4-1.4Z" fill="currentColor"/></svg></span>
        Rider
      </a>
    </nav>

    <div class="user-pill">
      <span class="avatar-sm" aria-hidden="true">MI</span>
      Muhammad Irfan
    </div>
  </aside>

  <!-- ---------- Main area ---------- -->
  <div class="main">
    <header class="page-header">
      <h1 id="pageTitle">Home</h1>
      <div class="header-right">
        <div id="updated" class="updated"><strong>Waiting for data…</strong>Delivery Rider 1</div>
        <span id="livePill" class="pill live-pill">Connecting</span>
      </div>
    </header>

    <div id="crashBanner" class="crash-banner" role="alert" hidden>
      <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M10.2 3.6 2.4 17.4c-.8 1.4.2 3.1 1.8 3.1h15.6c1.6 0 2.6-1.7 1.8-3.1L13.8 3.6c-.8-1.4-2.8-1.4-3.6 0Z" fill="currentColor"/><path class="knock" d="M12 8.8v4.6" stroke-width="2.3" stroke-linecap="round"/><circle class="knock-fill" cx="12" cy="16.8" r="1.35"/></svg>
      <div>
        <strong>Crash detected · Muhammad Irfan Bin Yusof</strong><br>
        <span id="crashBannerText">Impact — g</span>
      </div>
      <button class="btn reset-alert" type="button">I'm safe · Reset alert</button>
    </div>

    <main class="content">
      <!-- ======================= HOME ======================= -->
      <section id="page-home" class="page" aria-label="Home">
        <div class="split">
          <div class="split-panel">
            <h2 class="panel-title">Riders <span class="tag right">NEO-6M GPS</span></h2>

            <div class="rider-item">
              <span id="riderNum" class="rider-num">1</span>
              <div class="rider-info">
                <div class="rider-name">Muhammad Irfan Bin Yusof</div>
                <div class="rider-line">
                  <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M12 2.6a6.4 6.4 0 0 0-6.4 6.4c0 4.6 5.2 10.4 5.8 11.1.3.3.9.3 1.2 0 .6-.7 5.8-6.5 5.8-11.1A6.4 6.4 0 0 0 12 2.6Z" fill="currentColor"/><circle class="knock-fill" cx="12" cy="9" r="2.4"/></svg>
                  <span id="riderCoords">Locating…</span>
                </div>
                <div class="rider-line">
                  <svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="7.6" r="4.1" fill="currentColor"/><path d="M4.3 19.6c0-3.6 3.4-6.2 7.7-6.2s7.7 2.6 7.7 6.2c0 .8-.6 1.4-1.4 1.4H5.7c-.8 0-1.4-.6-1.4-1.4Z" fill="currentColor"/></svg>
                  Delivery Rider 1
                </div>
                <div class="rider-line">
                  <svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="12" r="9.2" fill="currentColor"/><path class="knock" d="M12 6.8V12l3.4 2.1" fill="none" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>
                  <span id="memberSub">Locating…</span>
                </div>
              </div>
              <span id="riderStatus" class="rider-status">Inactive</span>
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

            <h3 class="sub-title">Google Maps <span class="note">External URL Link</span></h3>
            <div id="mapsStatus" class="status info" role="status">Waiting for fresh GPS coordinates.</div>

            <a id="mapsLink" class="btn" target="_blank" rel="noopener noreferrer" aria-disabled="true" tabindex="-1">
              <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M8.6 4.2 3.7 6.1c-.4.2-.7.6-.7 1v12.1c0 .7.7 1.2 1.4.9l4.2-1.7V4.2ZM10.4 4.3v14.1l4.2 1.6V5.9l-4.2-1.6ZM16.4 5.8v14.1l4.9-1.9c.4-.2.7-.6.7-1V4.9c0-.7-.7-1.2-1.4-.9l-4.2 1.8Z" fill="currentColor"/></svg>
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
              The map and the Google Maps link need Internet access on this PC.
              Coordinates still update without it.
            </p>
          </div>

          <div class="map">
            <div id="homeMap" class="leaflet-map" role="region" aria-label="Map of the rider's location"></div>
            <div class="map-empty"><span class="map-msg">Loading map…</span></div>

            <div class="map-overlay top">
              <span class="map-chip"><span class="map-updated">Waiting for GPS fix</span> · OpenStreetMap</span>
              <button id="recenterMap" class="btn-small" type="button" disabled>
                <svg class="tint" viewBox="0 0 24 24" aria-hidden="true"><path d="M19.6 3.3 3.9 9.9c-.9.4-.8 1.7.2 1.9l6.5 1.5 1.5 6.5c.2 1 1.5 1.1 1.9.2l6.6-15.7c.3-.7-.4-1.4-1-1Z" fill="currentColor"/></svg>
                Recenter
              </button>
            </div>

            <div class="map-overlay bottom" role="group" aria-label="Map zoom">
              <button id="zoomIn" class="zoom-btn" type="button" aria-label="Zoom in">+</button>
              <span id="zoomLevel" class="zoom-level">17</span>
              <button id="zoomOut" class="zoom-btn" type="button" aria-label="Zoom out">−</button>
            </div>
          </div>
        </div>
      </section>

      <!-- ======================= ANALYTICS ======================= -->
      <section id="page-analytics" class="page" aria-label="Analytics" hidden>
        <div class="kpis">
          <div class="card">
            <div class="kpi-label">Impact force now</div>
            <div class="kpi-value"><span id="kpiG">—</span><small>g</small></div>
          </div>
          <div class="card">
            <div class="kpi-label">Peak impact (session)</div>
            <div class="kpi-value"><span id="kpiPeak">—</span><small>g</small></div>
          </div>
          <div class="card">
            <div class="kpi-label">Max lean (session)</div>
            <div class="kpi-value"><span id="kpiLean">—</span><small>°</small></div>
          </div>
          <div class="card">
            <div class="kpi-label">Crash alerts (session)</div>
            <div class="kpi-value"><span id="kpiAlerts">0</span></div>
          </div>
        </div>

        <div class="charts">
          <div class="card">
            <h2 class="card-title">
              Impact trend
              <div class="card-actions">
                <button id="exportCSV" class="btn-small" type="button">
                  <svg class="tint" viewBox="0 0 24 24" aria-hidden="true"><path d="M12 3.5v10.3M8.4 10.4 12 14l3.6-3.6M8 8.2H6.6a2.1 2.1 0 0 0-2.1 2.1v8.1c0 1.2.9 2.1 2.1 2.1h10.8c1.2 0 2.1-.9 2.1-2.1v-8.1c0-1.2-.9-2.1-2.1-2.1H16" fill="none" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"/></svg>
                  Export CSV
                </button>
              </div>
            </h2>
            <canvas id="impactChart" class="chart" role="img" aria-label="Impact force over the last two minutes"></canvas>
            <div class="legend">
              <span style="--swatch:#007aff">Impact force (g)</span>
              <span style="--swatch:#ff3b30">Crash threshold</span>
            </div>
          </div>

          <div class="card">
            <h2 class="card-title">Orientation trend</h2>
            <canvas id="orientChart" class="chart" role="img" aria-label="Lean and pitch angles over the last two minutes"></canvas>
            <div class="legend">
              <span style="--swatch:#007aff">Lean (roll) °</span>
              <span style="--swatch:#ff9500">Pitch °</span>
            </div>
          </div>
        </div>

        <div class="card">
          <h2 class="card-title">Sensor readings <span class="tag">MPU6050 Accelerometer</span></h2>
          <div id="mpuStatus" class="status info" role="status">Waiting for MPU6050 data…</div>
          <table class="table">
            <thead>
              <tr><th>Sensor</th><th>X axis</th><th>Y axis</th><th>Z axis</th><th>Unit</th></tr>
            </thead>
            <tbody>
              <tr>
                <td>Acceleration</td>
                <td id="ax" class="num">—</td><td id="ay" class="num">—</td><td id="az" class="num">—</td>
                <td>m/s²</td>
              </tr>
              <tr>
                <td>Angular velocity</td>
                <td id="gx" class="num">—</td><td id="gy" class="num">—</td><td id="gz" class="num">—</td>
                <td>rad/s</td>
              </tr>
            </tbody>
          </table>
          <p class="helper-text">Charts show the last 2 minutes of this browser session. Export CSV saves every reading since the page opened.</p>
        </div>
      </section>

      <!-- ======================= SAFETY ======================= -->
      <section id="page-safety" class="page" aria-label="Safety" hidden>
        <div class="split">
          <div class="split-panel">
            <h2 class="panel-title">Crash detection <span class="tag right">MPU6050</span></h2>

            <div id="safetyHero" class="safety-hero" role="status">
              <div class="safety-icon" aria-hidden="true">
                <svg viewBox="0 0 24 24"><path d="M12 3.2 5.2 5.9v5.6c0 4.2 2.8 7.7 6.8 8.9 4-1.2 6.8-4.7 6.8-8.9V5.9L12 3.2Z" fill="none" stroke="currentColor" stroke-width="1.9" stroke-linejoin="round"/><path d="M12 3.2 5.2 5.9v5.6c0 4.2 2.8 7.7 6.8 8.9Z" fill="currentColor"/></svg>
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

            <h3 class="sub-title">Alert settings</h3>
            <div>
              <div class="setting"><span>Crash threshold</span><span id="thresholdState" class="state on">2.5 g</span></div>
              <div class="setting"><span>Sound buzzer for 30 s on a crash</span><span class="state on">On</span></div>
              <div class="setting"><span>Send SMS with location to emergency contact</span><span class="state on">On</span></div>
              <div class="setting"><span>Emergency contact</span><span id="contact" class="state">—</span></div>
            </div>

            <h3 class="sub-title">Emergency response</h3>
            <div class="list">
              <div class="row"><span>Buzzer</span><strong id="buzzerState">Off</strong></div>
              <div class="row"><span>SMS alert</span><strong id="smsState">Standing by</strong></div>
              <div class="row"><span>GSM module</span><strong id="gsmState">—</strong></div>
            </div>

            <button id="resetAlert" class="btn coral reset-alert" type="button" disabled>I'm safe · Reset alert</button>
          </div>

          <div class="map">
            <div id="safetyMap" class="leaflet-map" role="region" aria-label="Map of the rider's location"></div>
            <div class="map-empty"><span class="map-msg">Loading map…</span></div>
            <div class="map-overlay top">
              <span class="map-chip">Rider location · <span class="map-updated">Waiting for GPS fix</span></span>
            </div>
          </div>
        </div>
      </section>

      <!-- ======================= VEHICLE ======================= -->
      <section id="page-vehicle" class="page" aria-label="Vehicle" hidden>
        <div class="vehicle-layout">
          <div class="card">
            <h2 class="card-title">Live vehicle orientation <span class="tag right">3D</span></h2>
            <div class="orient3d">
              <canvas id="orientCanvas" role="img" aria-label="3D model of the vehicle showing its live lean and pitch"></canvas>
              <div class="view-buttons" role="group" aria-label="Camera view">
                <button class="btn-small" type="button" data-view="three" aria-pressed="true">3D</button>
                <button class="btn-small" type="button" data-view="rear" aria-pressed="false">Rear</button>
                <button class="btn-small" type="button" data-view="side" aria-pressed="false">Side</button>
              </div>
              <div class="orient-hint">Drag to rotate · double-click to reset</div>
            </div>
          </div>

          <div class="card">
            <h2 class="card-title">Orientation</h2>
            <div class="grid2">
              <div class="attitude">
                <span class="label">Lean (roll)</span>
                <div id="rollValue" class="attitude-value">—</div>
                <div id="rollDir" class="attitude-dir">Waiting…</div>
              </div>
              <div class="attitude">
                <span class="label">Pitch</span>
                <div id="pitchValue" class="attitude-value">—</div>
                <div id="pitchDir" class="attitude-dir">Waiting…</div>
              </div>
            </div>

            <div id="orientStatus" class="status info" role="status">Waiting for orientation data…</div>

            <h3 class="sub-title">Acceleration <span class="note">m/s²</span></h3>
            <div class="list">
              <div class="row"><span class="axis-name axis-x">X axis</span><span id="vax" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-y">Y axis</span><span id="vay" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-z">Z axis</span><span id="vaz" class="axis-value">—</span></div>
            </div>

            <h3 class="sub-title">Angular velocity <span class="note">rad/s</span></h3>
            <div class="list">
              <div class="row"><span class="axis-name axis-x">X axis</span><span id="vgx" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-y">Y axis</span><span id="vgy" class="axis-value">—</span></div>
              <div class="row"><span class="axis-name axis-z">Z axis</span><span id="vgz" class="axis-value">—</span></div>
            </div>
          </div>
        </div>
      </section>

      <!-- ======================= RIDER ======================= -->
      <section id="page-rider" class="page" aria-label="Rider" hidden>
        <div class="card profile">
          <div id="profilePhoto" class="profile-photo" aria-hidden="true">MI</div>

          <div>
            <h2 class="profile-name">Muhammad Irfan Bin Yusof</h2>
            <div class="profile-ids">
              <span>
                <svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="7.6" r="4.1" fill="currentColor"/><path d="M4.3 19.6c0-3.6 3.4-6.2 7.7-6.2s7.7 2.6 7.7 6.2c0 .8-.6 1.4-1.4 1.4H5.7c-.8 0-1.4-.6-1.4-1.4Z" fill="currentColor"/></svg>
                Delivery Rider 1
              </span>
              <span>
                <svg viewBox="0 0 24 24" aria-hidden="true"><rect x="6.5" y="6.5" width="11" height="11" rx="2.2" fill="none" stroke="currentColor" stroke-width="1.8"/><rect x="9.5" y="9.5" width="5" height="5" rx="1" fill="currentColor"/><path d="M9.5 3.5v3M14.5 3.5v3M9.5 17.5v3M14.5 17.5v3M3.5 9.5h3M3.5 14.5h3M17.5 9.5h3M17.5 14.5h3" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"/></svg>
                ESP32-GPS
              </span>
            </div>

            <div class="info-row"><span>Role</span><span>Delivery Rider 1</span></div>
            <div class="info-row"><span>Motorcycle</span><span>Honda EX5</span></div>
            <div class="info-row"><span>Number plate</span><span>JBR 1240</span></div>
            <div class="info-row"><span>Emergency contact</span><span id="profileContact">—</span></div>
            <div class="info-row"><span>Dashboard address</span><span id="dashAddress">http://riderdashboard.local · http://192.168.4.1 (Wi-Fi ESP32-GPS)</span></div>
            <div class="info-row"><span>GPS module (NEO-6M)</span><span id="deviceGPS">—</span></div>
            <div class="info-row"><span>Motion sensor (MPU6050)</span><span id="deviceMPU">—</span></div>
            <div class="info-row"><span>GSM module (SIM800L)</span><span id="deviceGSM">—</span></div>
            <div class="info-row"><span>Last known location</span><span id="profileLocation">—</span></div>
          </div>

          <div class="stat-chips">
            <div class="stat-chip">
              <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M2.5 12.5h3.6l2.2-5.5 3.6 10.5 2.6-7.5 1.6 2.5h5.4" fill="none" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"/></svg>
              <div><strong id="statReadings">0</strong><span>Readings this session</span></div>
            </div>
            <div class="stat-chip coral">
              <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M10.2 3.6 2.4 17.4c-.8 1.4.2 3.1 1.8 3.1h15.6c1.6 0 2.6-1.7 1.8-3.1L13.8 3.6c-.8-1.4-2.8-1.4-3.6 0Z" fill="currentColor"/><path class="knock" d="M12 8.8v4.6" stroke-width="2.3" stroke-linecap="round"/><circle class="knock-fill" cx="12" cy="16.8" r="1.35"/></svg>
              <div><strong id="statAlerts">0</strong><span>Crash alerts this session</span></div>
            </div>
            <div class="stat-chip green">
              <svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="12" r="9.2" fill="currentColor"/><path class="knock" d="M12 6.8V12l3.4 2.1" fill="none" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/></svg>
              <div><strong id="statSession">0:00</strong><span>Session time</span></div>
            </div>
          </div>
        </div>
      </section>
    </main>
  </div>

<script>
  const POLL_MS = 1000;
  const METER_MAX_G = 4;
  const HISTORY_SECONDS = 120;
  const $ = (id) => document.getElementById(id);

  const PAGE_TITLES = {
    home: 'Home',
    analytics: 'Analytics',
    safety: 'Safety',
    vehicle: 'Vehicle',
    rider: 'Rider',
  };

  let currentPage = 'home';
  let lastFix = null;
  let fixSince = null;
  let alertActive = false;

  // Everything below is kept in this browser tab only, since the page opened.
  const session = {
    start: Date.now(),
    readings: 0,
    peakG: null,
    maxLean: null,
    alerts: 0,
    history: [],
  };

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

  // ---------- Pages ----------

  function showPage(name) {
    if (!PAGE_TITLES[name]) name = 'home';
    currentPage = name;

    Object.keys(PAGE_TITLES).forEach((page) => {
      $('page-' + page).hidden = page !== name;
    });

    document.querySelectorAll('.nav a').forEach((link) => {
      if (link.dataset.page === name) {
        link.setAttribute('aria-current', 'page');
      } else {
        link.removeAttribute('aria-current');
      }
    });

    $('pageTitle').textContent = PAGE_TITLES[name];

    // Canvases and maps on a hidden page have no size, so redraw them once they are visible.
    orient.dirty = true;
    drawCharts();
    maps.forEach((entry) => {
      entry.map.invalidateSize();
      if (lastFix && entry.follow) entry.map.setView(lastFix, entry.map.getZoom(), { animate: false });
    });
  }

  window.addEventListener('hashchange', () => showPage(location.hash.slice(1)));

  // ---------- Live map: Leaflet + OpenStreetMap (free, no API key) ----------
  // Leaflet loads from the Internet, so the page itself never waits for it when offline.

  const LEAFLET_CSS = 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css';
  const LEAFLET_JS = 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js';
  // Standard OpenStreetMap tiles: free and keyless for light use, as long as the credit stays visible.
  const TILE_URL = 'https://tile.openstreetmap.org/{z}/{x}/{y}.png';
  const TILE_ATTRIBUTION = '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors';

  const maps = [];
  let homeMap = null;

  function showMapMessage(text) {
    document.querySelectorAll('.map-msg').forEach((el) => {
      el.textContent = text;
      el.parentElement.hidden = false;
    });
  }

  function riderIcon() {
    return L.divIcon({
      className: 'rider-pin' + (alertActive ? ' crash' : ''),
      html: '<span><b>MI</b></span>',
      iconSize: [40, 40],
      iconAnchor: [20, 48],
    });
  }

  function createMap(id) {
    const map = L.map(id, { zoomControl: false }).setView([3.139, 101.6869], 12);
    L.tileLayer(TILE_URL, { maxZoom: 19, attribution: TILE_ATTRIBUTION }).addTo(map);

    const entry = { map, marker: null, follow: true };

    // Dragging the map stops it following the rider until Recenter is pressed.
    map.on('dragstart', () => { entry.follow = false; });
    maps.push(entry);
    return entry;
  }

  function placeRider(firstFix) {
    if (!maps.length) return;

    if (!lastFix) {
      showMapMessage('Waiting for GPS fix…');
      return;
    }

    document.querySelectorAll('.map-empty').forEach((el) => { el.hidden = true; });

    maps.forEach((entry) => {
      if (entry.marker) {
        entry.marker.setLatLng(lastFix);
      } else {
        entry.marker = L.marker(lastFix, { icon: riderIcon(), keyboard: false }).addTo(entry.map);
      }

      if (firstFix) {
        entry.map.setView(lastFix, 17, { animate: false });
      } else if (entry.follow) {
        entry.map.panTo(lastFix);
      }
    });
  }

  const LEAFLET_RETRY_MS = 10000;

  function loadLeaflet() {
    // A fresh <link> each try, so the styles also arrive if an earlier try was offline.
    const css = document.createElement('link');
    css.rel = 'stylesheet';
    css.href = LEAFLET_CSS;
    document.head.appendChild(css);

    const script = document.createElement('script');
    script.src = LEAFLET_JS;

    script.onload = () => {
      homeMap = createMap('homeMap');
      createMap('safetyMap');
      homeMap.map.on('zoomend', () => { $('zoomLevel').textContent = homeMap.map.getZoom(); });
      $('zoomLevel').textContent = homeMap.map.getZoom();
      placeRider(true);
    };

    // Offline right now (for example on the ESP32-GPS Wi-Fi): keep trying, so the map
    // appears by itself once this PC has Internet again.
    script.onerror = () => {
      script.remove();
      css.remove();
      showMapMessage('The map needs Internet access on this PC. Retrying every 10 seconds; ' +
                     'coordinates and alerts still work without it.');
      setTimeout(loadLeaflet, LEAFLET_RETRY_MS);
    };

    document.head.appendChild(script);
  }

  $('zoomIn').addEventListener('click', () => { if (homeMap) homeMap.map.zoomIn(); });
  $('zoomOut').addEventListener('click', () => { if (homeMap) homeMap.map.zoomOut(); });

  $('recenterMap').addEventListener('click', () => {
    if (!homeMap || !lastFix) return;
    homeMap.follow = true;
    homeMap.map.setView(lastFix, Math.max(homeMap.map.getZoom(), 15));
  });

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

  // ---------- Analytics charts ----------

  function sizeCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const w = Math.round(canvas.clientWidth * dpr);
    const h = Math.round(canvas.clientHeight * dpr);
    if (!w || !h) return null;

    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }

    return { ctx: canvas.getContext('2d'), w, h, dpr };
  }

  function drawLineChart(canvas, series, options) {
    const size = sizeCanvas(canvas);
    if (!size) return;

    const { ctx, w, h, dpr } = size;
    const left = 44 * dpr;
    const right = 10 * dpr;
    const top = 10 * dpr;
    const bottom = 26 * dpr;
    const plotW = w - left - right;
    const plotH = h - top - bottom;
    const now = Date.now();

    let min = options.min;
    let max = options.max;
    series.forEach((s) => s.points.forEach((p) => {
      min = Math.min(min, p.v);
      max = Math.max(max, p.v);
    }));

    const x = (t) => left + plotW * (1 - (now - t) / (HISTORY_SECONDS * 1000));
    const y = (v) => top + plotH * (1 - (v - min) / (max - min));

    ctx.clearRect(0, 0, w, h);
    ctx.font = (11 * dpr) + 'px ' + getComputedStyle(document.body).fontFamily;
    ctx.fillStyle = '#8e8e93';
    ctx.strokeStyle = 'rgba(60, 60, 67, 0.12)';
    ctx.lineWidth = dpr;

    // Horizontal grid with five labelled steps.
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 4; i++) {
      const v = min + ((max - min) * i) / 4;
      ctx.beginPath();
      ctx.moveTo(left, y(v));
      ctx.lineTo(w - right, y(v));
      ctx.stroke();
      ctx.fillText(v.toFixed(options.digits), left - 8 * dpr, y(v));
    }

    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    [120, 90, 60, 30, 0].forEach((s) => {
      ctx.fillText(s ? '-' + s + ' s' : 'now', x(now - s * 1000), h - bottom + 8 * dpr);
    });

    if (typeof options.threshold === 'number') {
      ctx.save();
      ctx.setLineDash([6 * dpr, 5 * dpr]);
      ctx.strokeStyle = '#ff3b30';
      ctx.lineWidth = 1.5 * dpr;
      ctx.beginPath();
      ctx.moveTo(left, y(options.threshold));
      ctx.lineTo(w - right, y(options.threshold));
      ctx.stroke();
      ctx.restore();
    }

    series.forEach((s, index) => {
      const pts = s.points.map((p) => [x(p.t), y(p.v)]);
      if (pts.length < 2) return;

      // Smooth curve through the points, like the reference trend chart.
      ctx.beginPath();
      ctx.moveTo(pts[0][0], pts[0][1]);
      for (let i = 1; i < pts.length; i++) {
        const mx = (pts[i - 1][0] + pts[i][0]) / 2;
        ctx.bezierCurveTo(mx, pts[i - 1][1], mx, pts[i][1], pts[i][0], pts[i][1]);
      }

      if (index === 0) {
        ctx.save();
        const gradient = ctx.createLinearGradient(0, top, 0, h - bottom);
        gradient.addColorStop(0, s.color + '33');
        gradient.addColorStop(1, s.color + '00');
        ctx.lineTo(pts[pts.length - 1][0], h - bottom);
        ctx.lineTo(pts[0][0], h - bottom);
        ctx.closePath();
        ctx.fillStyle = gradient;
        ctx.fill();
        ctx.restore();

        ctx.beginPath();
        ctx.moveTo(pts[0][0], pts[0][1]);
        for (let i = 1; i < pts.length; i++) {
          const mx = (pts[i - 1][0] + pts[i][0]) / 2;
          ctx.bezierCurveTo(mx, pts[i - 1][1], mx, pts[i][1], pts[i][0], pts[i][1]);
        }
      }

      ctx.strokeStyle = s.color;
      ctx.lineWidth = 2.5 * dpr;
      ctx.lineJoin = 'round';
      ctx.stroke();
    });
  }

  function drawCharts() {
    if (currentPage !== 'analytics') return;

    const since = Date.now() - HISTORY_SECONDS * 1000;
    const recent = session.history.filter((r) => r.t >= since && typeof r.g === 'number');
    const threshold = session.threshold || 2.5;

    drawLineChart($('impactChart'), [
      { color: '#007aff', points: recent.map((r) => ({ t: r.t, v: r.g })) },
    ], { min: 0, max: Math.max(METER_MAX_G, threshold + 0.5), threshold, digits: 1 });

    drawLineChart($('orientChart'), [
      { color: '#007aff', points: recent.filter((r) => typeof r.roll === 'number').map((r) => ({ t: r.t, v: r.roll })) },
      { color: '#ff9500', points: recent.filter((r) => typeof r.pitch === 'number').map((r) => ({ t: r.t, v: r.pitch })) },
    ], { min: -45, max: 45, digits: 0 });
  }

  window.addEventListener('resize', drawCharts);

  $('exportCSV').addEventListener('click', () => {
    const header = 'time,impact_g,roll_deg,pitch_deg,ax,ay,az,gx,gy,gz,latitude,longitude,crash_alert';
    const cell = (v) => (typeof v === 'number' && isFinite(v) ? v : '');
    const rows = session.history.map((r) => [
      new Date(r.t).toISOString(), cell(r.g), cell(r.roll), cell(r.pitch),
      cell(r.ax), cell(r.ay), cell(r.az), cell(r.gx), cell(r.gy), cell(r.gz),
      cell(r.lat), cell(r.lng), r.alert ? 1 : 0,
    ].join(','));

    const blob = new Blob([header + '\n' + rows.join('\n')], { type: 'text/csv' });
    const link = document.createElement('a');
    const stamp = new Date().toISOString().slice(0, 16).replace(/[-:T]/g, '');
    link.href = URL.createObjectURL(blob);
    link.download = 'rider-session-' + stamp + '.csv';
    link.click();
    setTimeout(() => URL.revokeObjectURL(link.href), 1000);
  });

  // ---------- 3D vehicle orientation ----------
  // A tiny canvas renderer instead of a 3D library, so it also works on the offline ESP32 Wi-Fi.
  // World axes: x = forward, y = left, z = up; the vehicle pivots on the ground under its centre.

  const VIEWS = {
    three: { az: Math.PI - 0.75, el: 0.3 },
    rear: { az: 0, el: 0.12 },
    side: { az: -Math.PI / 2, el: 0.08 },
  };

  const orient = {
    canvas: $('orientCanvas'),
    roll: 0,
    pitch: 0,
    targetRoll: 0,
    targetPitch: 0,
    hasData: false,
    az: VIEWS.three.az,
    el: VIEWS.three.el,
    drag: null,
    dirty: true,
  };

  const LIGHT = (() => {
    const v = [0.35, 0.45, 0.82];
    const len = Math.hypot(v[0], v[1], v[2]);
    return v.map((n) => n / len);
  })();

  function box(centre, size, color, accent) {
    const [x, y, z] = centre;
    const [a, b, h] = size.map((n) => n / 2);
    const v = [
      [x - a, y - b, z - h], [x + a, y - b, z - h], [x + a, y + b, z - h], [x - a, y + b, z - h],
      [x - a, y - b, z + h], [x + a, y - b, z + h], [x + a, y + b, z + h], [x - a, y + b, z + h],
    ];
    return [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [3, 0, 4, 7]]
      .map((face) => ({ pts: face.map((i) => v[i]), centre, color, accent }));
  }

  // A cylinder lying across the bike (along y), for tyres and hubs.
  function cylinder(cx, cz, r, width, sideColor, capColor) {
    const n = 18;
    const left = [];
    const right = [];
    const faces = [];
    const centre = [cx, 0, cz];

    for (let i = 0; i < n; i++) {
      const t = (i / n) * Math.PI * 2;
      left.push([cx + r * Math.cos(t), width / 2, cz + r * Math.sin(t)]);
      right.push([cx + r * Math.cos(t), -width / 2, cz + r * Math.sin(t)]);
    }

    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      faces.push({ pts: [left[i], left[j], right[j], right[i]], centre, color: sideColor });
    }

    faces.push({ pts: left, centre, color: capColor }, { pts: right, centre, color: capColor });
    return faces;
  }

  // Spoked wheel: dark tyre with a silver rim face and a dark hub.
  function wheel(cx, r, width) {
    return [
      ...cylinder(cx, r, r, width, '#24262b', '#c9ced6'),
      ...cylinder(cx, r, r * 0.22, width * 1.6, '#3f444d', '#6b7280'),
    ];
  }

  // Honda EX5-style underbone motorcycle with a green delivery box on the rear rack,
  // in metres-ish units. The delivery box (accent) turns red during a crash alert.
  const RED = '#c8102e';
  const WHITE = '#eef0f2';
  const BLACK = '#1f2329';
  const SILVER = '#b8bec7';

  const VEHICLE = [
    // Wheels and suspension
    ...wheel(-0.6, 0.28, 0.1),
    ...wheel(0.64, 0.28, 0.1),
    ...box([0.6, 0.08, 0.56], [0.05, 0.04, 0.56], SILVER),
    ...box([0.6, -0.08, 0.56], [0.05, 0.04, 0.56], SILVER),
    ...box([-0.56, 0.12, 0.47], [0.05, 0.04, 0.32], BLACK),
    ...box([-0.56, -0.12, 0.47], [0.05, 0.04, 0.32], BLACK),

    // Mudguards
    ...box([0.64, 0, 0.6], [0.44, 0.13, 0.05], RED),
    ...box([-0.64, 0, 0.6], [0.4, 0.14, 0.05], RED),

    // Front: red centre column, white leg shields, handlebar cowl with headlight
    ...box([0.44, 0, 0.66], [0.14, 0.14, 0.62], RED),
    ...box([0.42, 0.15, 0.6], [0.1, 0.16, 0.46], WHITE),
    ...box([0.42, -0.15, 0.6], [0.1, 0.16, 0.46], WHITE),
    ...box([0.5, 0, 1.03], [0.24, 0.34, 0.14], RED),
    ...box([0.63, 0, 1.04], [0.03, 0.14, 0.08], '#fff3c4'),
    ...box([0.6, 0.15, 1.03], [0.05, 0.06, 0.05], '#ffb020'),
    ...box([0.6, -0.15, 1.03], [0.05, 0.06, 0.05], '#ffb020'),

    // Handlebar grips and mirrors
    ...box([0.48, 0.26, 1.03], [0.05, 0.18, 0.04], BLACK),
    ...box([0.48, -0.26, 1.03], [0.05, 0.18, 0.04], BLACK),
    ...box([0.5, 0.2, 1.17], [0.02, 0.02, 0.2], BLACK),
    ...box([0.5, -0.2, 1.17], [0.02, 0.02, 0.2], BLACK),
    ...box([0.5, 0.23, 1.28], [0.03, 0.09, 0.05], BLACK),
    ...box([0.5, -0.23, 1.28], [0.03, 0.09, 0.05], BLACK),

    // Step-through floor, engine and exhaust
    ...box([0.12, 0, 0.42], [0.5, 0.18, 0.07], WHITE),
    ...box([-0.02, 0, 0.3], [0.32, 0.22, 0.2], '#6b7280'),
    ...box([-0.3, -0.15, 0.24], [0.62, 0.06, 0.06], SILVER),

    // Rear body: red side covers with yellow pinstripe, black seat, rack and tail light
    ...box([-0.3, 0, 0.62], [0.64, 0.3, 0.2], RED),
    ...box([-0.3, 0.152, 0.66], [0.5, 0.006, 0.025], '#f5c400'),
    ...box([-0.3, -0.152, 0.66], [0.5, 0.006, 0.025], '#f5c400'),
    ...box([-0.28, 0, 0.76], [0.64, 0.28, 0.08], BLACK),
    ...box([-0.74, 0, 0.8], [0.38, 0.3, 0.03], '#4b5263'),
    ...box([-0.94, 0, 0.67], [0.03, 0.12, 0.06], '#ff3b30'),

    // Rider: green delivery jacket with a yellow-green hem, dark trousers,
    // green helmet with a white visor, hands on the grips and feet on the floorboard.
    ...box([-0.18, 0, 1.08], [0.24, 0.34, 0.5], '#1fae4b'),
    ...box([-0.18, 0, 0.86], [0.246, 0.346, 0.04], '#b7d433'),
    ...box([0.17, 0.21, 1.15], [0.62, 0.08, 0.08], '#1fae4b'),
    ...box([0.17, -0.21, 1.15], [0.62, 0.08, 0.08], '#1fae4b'),
    ...box([0.46, 0.26, 1.06], [0.07, 0.07, 0.06], '#e0b088'),
    ...box([0.46, -0.26, 1.06], [0.07, 0.07, 0.06], '#e0b088'),
    ...box([-0.14, 0, 1.4], [0.16, 0.16, 0.16], '#e0b088'),
    ...box([-0.15, 0, 1.5], [0.25, 0.25, 0.15], '#18a84a'),
    ...box([-0.01, 0, 1.45], [0.06, 0.23, 0.03], '#ffffff'),
    ...box([-0.02, 0.13, 0.84], [0.36, 0.11, 0.12], '#2b2f3a'),
    ...box([-0.02, -0.13, 0.84], [0.36, 0.11, 0.12], '#2b2f3a'),
    ...box([0.14, 0.14, 0.62], [0.1, 0.1, 0.34], '#2b2f3a'),
    ...box([0.14, -0.14, 0.62], [0.1, 0.1, 0.34], '#2b2f3a'),
    ...box([0.19, 0.14, 0.48], [0.17, 0.1, 0.06], BLACK),
    ...box([0.19, -0.14, 0.48], [0.17, 0.1, 0.06], BLACK),

    // Delivery box: green with a reflective band and a front pocket facing back
    ...box([-0.74, 0, 1.06], [0.5, 0.5, 0.48], '#00b14f', true),
    ...box([-0.74, 0, 1.22], [0.506, 0.506, 0.04], '#dfe3e8'),
    ...box([-0.993, 0, 1.0], [0.012, 0.38, 0.26], '#009a44', true),
  ];

  function rotateVehicle(p, cr, sr, cp, sp) {
    // Roll about the forward axis (lean right = positive), then pitch about the left axis (nose up = positive).
    const y1 = p[1] * cr - p[2] * sr;
    const z1 = p[1] * sr + p[2] * cr;
    return [p[0] * cp - z1 * sp, y1, p[0] * sp + z1 * cp];
  }

  function normalOf(pts) {
    let nx = 0;
    let ny = 0;
    let nz = 0;

    for (let i = 0; i < pts.length; i++) {
      const a = pts[i];
      const b = pts[(i + 1) % pts.length];
      nx += (a[1] - b[1]) * (a[2] + b[2]);
      ny += (a[2] - b[2]) * (a[0] + b[0]);
      nz += (a[0] - b[0]) * (a[1] + b[1]);
    }

    const len = Math.hypot(nx, ny, nz) || 1;
    return [nx / len, ny / len, nz / len];
  }

  function shade(hex, light) {
    const n = parseInt(hex.slice(1), 16);
    const k = Math.min(1.15, light);
    return 'rgb(' + Math.round(((n >> 16) & 255) * k) + ',' +
           Math.round(((n >> 8) & 255) * k) + ',' + Math.round((n & 255) * k) + ')';
  }

  // Returns false when the canvas is hidden, so the caller tries again later.
  function drawVehicle() {
    const size = sizeCanvas(orient.canvas);
    if (!size) return false;

    const { ctx, w, h, dpr } = size;
    ctx.clearRect(0, 0, w, h);

    // Follow the vehicle's middle part-way, so a big lean or a tip-over stays in frame.
    const rollRad = (orient.roll * Math.PI) / 180;
    const pitchRad = (orient.pitch * Math.PI) / 180;
    const target = [0, -Math.sin(rollRad) * 0.5, 0.35 + Math.cos(rollRad) * 0.37];
    const dist = 4.4;
    const focal = Math.min(h * 1.75, w * 1.1);
    const cx = w / 2;
    const cy = h * 0.56;
    const ca = Math.cos(orient.az);
    const sa = Math.sin(orient.az);
    const ce = Math.cos(orient.el);
    const se = Math.sin(orient.el);

    // Camera looks along `dir`, tilted down by the elevation angle.
    const dir = [ce * ca, -ce * sa, -se];
    const camera = [target[0] - dist * dir[0], target[1] - dist * dir[1], target[2] - dist * dir[2]];

    function project(p) {
      const x = p[0] - target[0];
      const y = p[1] - target[1];
      const z = p[2] - target[2];
      const x1 = x * ca - y * sa;
      const y1 = x * sa + y * ca;
      const depth = x1 * ce - z * se + dist;
      const up = x1 * se + z * ce;
      return [cx - (y1 * focal) / depth, cy - (up * focal) / depth, depth];
    }

    function polygon(points, fill, stroke) {
      ctx.beginPath();
      points.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
      ctx.closePath();
      if (fill) {
        ctx.fillStyle = fill;
        ctx.fill();
      }
      if (stroke) {
        ctx.strokeStyle = stroke;
        ctx.stroke();
      }
    }

    // Ground grid, fixed in the world so the lean is easy to read against it.
    ctx.lineWidth = dpr;
    ctx.strokeStyle = 'rgba(0, 122, 255, 0.18)';
    for (let i = -2; i <= 2; i += 0.5) {
      const a = project([i, -2, 0]);
      const b = project([i, 2, 0]);
      const c = project([-2, i, 0]);
      const d = project([2, i, 0]);
      ctx.beginPath();
      ctx.moveTo(a[0], a[1]);
      ctx.lineTo(b[0], b[1]);
      ctx.moveTo(c[0], c[1]);
      ctx.lineTo(d[0], d[1]);
      ctx.stroke();
    }

    // Soft shadow that slides sideways as the vehicle leans.
    const shadowY = -Math.sin(rollRad) * 0.7;
    const shadow = [];
    for (let i = 0; i < 24; i++) {
      const t = (i / 24) * Math.PI * 2;
      shadow.push(project([Math.cos(t) * 1.05, shadowY + Math.sin(t) * 0.32, 0]));
    }
    polygon(shadow, 'rgba(20, 30, 50, 0.12)');

    const cr = Math.cos(rollRad);
    const sr = Math.sin(rollRad);
    const cp = Math.cos(pitchRad);
    const sp = Math.sin(pitchRad);
    const faces = [];

    VEHICLE.forEach((face) => {
      const pts = face.pts.map((p) => rotateVehicle(p, cr, sr, cp, sp));
      const centre = rotateVehicle(face.centre, cr, sr, cp, sp);
      const mid = [0, 0, 0];
      pts.forEach((p) => { mid[0] += p[0]; mid[1] += p[1]; mid[2] += p[2]; });
      mid[0] /= pts.length;
      mid[1] /= pts.length;
      mid[2] /= pts.length;

      // Point the normal away from the part's centre, then drop faces turned away from the camera.
      let n = normalOf(pts);
      if (n[0] * (mid[0] - centre[0]) + n[1] * (mid[1] - centre[1]) + n[2] * (mid[2] - centre[2]) < 0) {
        n = n.map((v) => -v);
      }
      if (n[0] * (mid[0] - camera[0]) + n[1] * (mid[1] - camera[1]) + n[2] * (mid[2] - camera[2]) >= 0) {
        return;
      }

      const light = 0.55 + 0.6 * Math.max(0, n[0] * LIGHT[0] + n[1] * LIGHT[1] + n[2] * LIGHT[2]);
      const base = face.accent && alertActive ? '#ff3b30' : face.color;
      faces.push({ screen: pts.map(project), depth: project(mid)[2], fill: shade(base, light) });
    });

    // Painter's algorithm: far faces first.
    faces.sort((a, b) => b.depth - a.depth);
    ctx.globalAlpha = orient.hasData ? 1 : 0.4;
    ctx.lineJoin = 'round';
    faces.forEach((face) => polygon(face.screen, face.fill, face.fill));
    ctx.globalAlpha = 1;
    return true;
  }

  function animateVehicle() {
    const dr = orient.targetRoll - orient.roll;
    const dp = orient.targetPitch - orient.pitch;

    // Ease toward the latest reading so the 1-second updates look smooth.
    if (orient.dirty || Math.abs(dr) > 0.05 || Math.abs(dp) > 0.05) {
      orient.roll += dr * 0.12;
      orient.pitch += dp * 0.12;
      orient.dirty = !drawVehicle();
    }

    requestAnimationFrame(animateVehicle);
  }

  function setVehicleOrientation(roll, pitch) {
    orient.hasData = typeof roll === 'number' && typeof pitch === 'number';
    orient.targetRoll = orient.hasData ? Math.max(-120, Math.min(120, roll)) : 0;
    orient.targetPitch = orient.hasData ? Math.max(-90, Math.min(90, pitch)) : 0;
    orient.dirty = true;
  }

  function setView(name) {
    orient.az = VIEWS[name].az;
    orient.el = VIEWS[name].el;
    orient.dirty = true;
    document.querySelectorAll('.view-buttons button').forEach((b) => {
      b.setAttribute('aria-pressed', b.dataset.view === name ? 'true' : 'false');
    });
  }

  document.querySelectorAll('.view-buttons button').forEach((b) => {
    b.addEventListener('click', () => setView(b.dataset.view));
  });

  orient.canvas.addEventListener('pointerdown', (event) => {
    orient.drag = { x: event.clientX, y: event.clientY };
    orient.canvas.setPointerCapture(event.pointerId);
    orient.canvas.classList.add('dragging');
  });

  orient.canvas.addEventListener('pointermove', (event) => {
    if (!orient.drag) return;
    orient.az -= (event.clientX - orient.drag.x) * 0.01;
    orient.el = Math.max(0, Math.min(1.35, orient.el + (event.clientY - orient.drag.y) * 0.01));
    orient.drag = { x: event.clientX, y: event.clientY };
    orient.dirty = true;
    document.querySelectorAll('.view-buttons button').forEach((b) => b.setAttribute('aria-pressed', 'false'));
  });

  ['pointerup', 'pointercancel'].forEach((type) => {
    orient.canvas.addEventListener(type, () => {
      orient.drag = null;
      orient.canvas.classList.remove('dragging');
    });
  });

  orient.canvas.addEventListener('dblclick', () => setView('three'));
  window.addEventListener('resize', () => { orient.dirty = true; });
  requestAnimationFrame(animateVehicle);

  // ---------- Live data ----------

  function updateMap(d) {
    if (!d.valid) {
      if (!lastFix && maps.length) showMapMessage('Waiting for GPS fix…');
      return;
    }

    const firstFix = !lastFix;
    lastFix = [d.latitude, d.longitude];
    $('recenterMap').disabled = false;

    document.querySelectorAll('.map-updated').forEach((el) => {
      el.textContent = (d.fresh ? 'Live · ' : 'Last known · ') + clock(Date.now(), true);
    });

    // The pin moves with every reading; the map follows unless the user has dragged it.
    placeRider(firstFix);
  }

  function updateGPS(d) {
    const lat = $('latitude');
    const lng = $('longitude');

    lat.textContent = d.valid ? fmt(d.latitude, 6) : '—';
    lng.textContent = d.valid ? fmt(d.longitude, 6) : '—';
    lat.classList.toggle('stale', d.valid && !d.fresh);
    lng.classList.toggle('stale', d.valid && !d.fresh);

    const coords = d.valid ? fmt(d.latitude, 5) + ', ' + fmt(d.longitude, 5) : 'Locating…';
    $('riderCoords').textContent = coords;
    $('profileLocation').textContent = d.valid ? coords + (d.fresh ? ' (live)' : ' (stale)') : 'No GPS fix yet';

    if (d.fresh) {
      if (!fixSince) fixSince = Date.now();
    } else {
      fixSince = null;
    }

    let gpsText;

    if (!d.receiving) {
      gpsText = 'No data from the GPS module. Check wiring (GPS TX → GPIO16).';
      setStatus('gpsStatus', 'error', gpsText);
      $('memberSub').textContent = 'GPS offline';
    } else if (!d.valid) {
      gpsText = 'GPS connected, waiting for a satellite fix…';
      setStatus('gpsStatus', 'warn', gpsText);
      $('memberSub').textContent = 'Locating…';
    } else if (!d.fresh) {
      gpsText = 'GPS fix is stale (last update ' + Math.round(d.age_ms / 1000) + ' s ago).';
      setStatus('gpsStatus', 'warn', gpsText);
      $('memberSub').textContent = 'Last known location';
    } else {
      gpsText = 'Live GPS fix';
      setStatus('gpsStatus', 'ok', gpsText);
      $('memberSub').textContent = 'Live since ' + clock(fixSince, false);
    }

    $('deviceGPS').textContent = gpsText;

    if (!alertActive) {
      $('riderNum').className = 'rider-num' + (d.fresh ? ' live' : '');
      $('riderStatus').className = 'rider-status' + (d.fresh ? ' live' : '');
      $('riderStatus').textContent = d.fresh ? 'Active' : 'Inactive';
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
    let mpuText;

    if (!m || !m.detected) {
      mpuText = 'MPU6050 not detected. Check SDA → GPIO21 and SCL → GPIO22.';
      setStatus('mpuStatus', 'error', mpuText);
    } else if (!m.valid) {
      mpuText = 'MPU6050 detected, but the last reading failed.';
      setStatus('mpuStatus', 'warn', mpuText);
    } else {
      mpuText = 'Live MPU6050 data';
      setStatus('mpuStatus', 'ok', mpuText);
    }

    $('deviceMPU').textContent = mpuText;

    const valid = m && m.valid;
    ['ax', 'ay', 'az', 'gx', 'gy', 'gz'].forEach((id) => {
      const text = valid ? fmt(m[id], 2) : '—';
      $(id).textContent = text;
      $('v' + id).textContent = text;
    });

    $('kpiG').textContent = valid ? fmt(m.g, 2) : '—';

    updateOrientation(valid ? m.roll : null, valid ? m.pitch : null);
  }

  function updateOrientation(roll, pitch) {
    if (typeof roll !== 'number' || typeof pitch !== 'number') {
      ['rollValue', 'pitchValue'].forEach((id) => { $(id).textContent = '—'; });
      ['rollDir', 'pitchDir'].forEach((id) => { $(id).textContent = 'Waiting…'; });
      setStatus('orientStatus', 'info', 'Waiting for orientation data…');
      setVehicleOrientation(null, null);
      return;
    }

    setVehicleOrientation(roll, pitch);

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
    session.threshold = threshold;

    const markPercent = Math.min(threshold / METER_MAX_G, 1) * 100 + '%';
    $('gMark').style.left = markPercent;
    $('thresholdLabel').style.left = markPercent;
    $('thresholdLabel').textContent = fmt(threshold, 1) + ' g';
    $('thresholdState').textContent = fmt(threshold, 1) + ' g';

    const g = (m && m.valid) ? m.g : null;
    $('gNow').textContent = fmt(g, 2);

    const fill = $('gFill');
    fill.style.width = g === null ? '0' : Math.min(g / METER_MAX_G, 1) * 100 + '%';
    fill.style.background = g === null || g < threshold * 0.6 ? 'var(--green)'
                          : g < threshold ? 'var(--amber)' : 'var(--red)';

    // Count each new crash once, and turn the 3D vehicle and map marker red.
    if (a.active && !alertActive) session.alerts += 1;
    const alertChanged = alertActive !== a.active;
    alertActive = a.active;

    if (alertChanged) {
      orient.dirty = true;
      maps.forEach((entry) => { if (entry.marker) entry.marker.setIcon(riderIcon()); });
    }

    const buzzerText = a.buzzer ? 'Sounding' : (a.active ? 'Stopped' : 'Off');
    const smsText = { idle: 'Standing by', sending: 'Sending…', sent: 'Sent', failed: 'Failed' }[a.sms] || a.sms;

    $('safetyHero').classList.toggle('alert', a.active);
    $('crashBanner').hidden = !a.active;
    $('navBadge').hidden = !a.active;
    document.querySelectorAll('.reset-alert').forEach((b) => { b.disabled = !a.active; });

    if (a.active) {
      const impact = 'Impact ' + fmt(a.impact_g, 1) + ' g · ' + a.seconds_ago + ' s ago';
      $('safetyTitle').textContent = 'Crash detected';
      $('safetySub').textContent = impact;
      $('crashBannerText').textContent = impact + ' · Buzzer ' + buzzerText.toLowerCase() +
                                         ' · SMS ' + smsText.toLowerCase();
      $('riderNum').className = 'rider-num crash';
      $('riderStatus').className = 'rider-status crash';
      $('riderStatus').textContent = 'Crash';
    } else {
      $('safetyTitle').textContent = 'Rider is safe';
      $('safetySub').textContent = 'Monitoring the MPU6050 for impacts.';
    }

    const contact = a.contact || '—';
    const gsmText = a.gsm_ready ? 'SIM800L ready' : 'Not responding';

    $('buzzerState').textContent = buzzerText;
    $('smsState').textContent = a.sms_detail ? smsText + ' · ' + a.sms_detail : smsText;
    $('contact').textContent = contact;
    $('profileContact').textContent = contact;
    $('gsmState').textContent = gsmText;
    $('deviceGSM').textContent = gsmText;
  }

  function recordReading(d) {
    const m = d.mpu || {};
    const valid = m.valid;

    session.readings += 1;
    session.history.push({
      t: Date.now(),
      g: valid ? m.g : null,
      roll: valid ? m.roll : null,
      pitch: valid ? m.pitch : null,
      ax: valid ? m.ax : null,
      ay: valid ? m.ay : null,
      az: valid ? m.az : null,
      gx: valid ? m.gx : null,
      gy: valid ? m.gy : null,
      gz: valid ? m.gz : null,
      lat: d.valid ? d.latitude : null,
      lng: d.valid ? d.longitude : null,
      alert: d.alert && d.alert.active,
    });

    // About 10 hours at one reading per second; older readings drop off the CSV export.
    if (session.history.length > 36000) session.history.shift();

    if (valid && typeof m.g === 'number') {
      session.peakG = Math.max(session.peakG || 0, m.g);
    }
    if (valid && typeof m.roll === 'number') {
      session.maxLean = Math.max(session.maxLean || 0, Math.abs(m.roll));
    }

    $('kpiPeak').textContent = fmt(session.peakG, 2);
    $('kpiLean').textContent = fmt(session.maxLean, 0);
    $('kpiAlerts').textContent = session.alerts;
    $('statReadings').textContent = session.readings;
    $('statAlerts').textContent = session.alerts;
  }

  setInterval(() => {
    const seconds = Math.floor((Date.now() - session.start) / 1000);
    const h = Math.floor(seconds / 3600);
    const m = Math.floor((seconds % 3600) / 60);
    const s = String(seconds % 60).padStart(2, '0');
    $('statSession').textContent = h ? h + ':' + String(m).padStart(2, '0') + ':' + s : m + ':' + s;
  }, 1000);

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
      $('updated').innerHTML = '<strong>Updated ' + clock(Date.now(), true) + '</strong>Delivery Rider 1';

      updateAlert(data.alert, data.mpu);
      updateGPS(data);
      updateMPU(data.mpu);
      recordReading(data);

      $('dashAddress').textContent = 'http://riderdashboard.local · ' +
        (data.router_ip ? 'http://' + data.router_ip + ' (hotspot) · ' : '') +
        'http://192.168.4.1 (Wi-Fi ESP32-GPS)';
      drawCharts();
    } catch (error) {
      showOffline();
    } finally {
      clearTimeout(timer);
      setTimeout(poll, POLL_MS);
    }
  }

  showPage(location.hash.slice(1));
  loadLeaflet();
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
  smsIndex = 0;
  smsState = SMS_WAIT_AT;
  gsmCommand("AT");
}

// Starts sending smsMessages[smsIndex]; the SIM800L answers with a '>' prompt.
void beginSMSMessage() {
  gsmBuffer = "";
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(EMERGENCY_NUMBER);
  gsmSerial.print("\"\r");
  smsStepStart = millis();
  smsState = SMS_WAIT_PROMPT;
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
        beginSMSMessage();
      } else if (gsmBuffer.indexOf("ERROR") >= 0 || elapsed > 3000) {
        smsFail("SIM800L rejected text mode");
      }
      break;

    case SMS_WAIT_PROMPT:
      if (gsmBuffer.indexOf('>') >= 0) {
        gsmBuffer = "";
        gsmSerial.print(smsMessages[smsIndex]);
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
        smsIndex++;

        if (smsIndex < SMS_MESSAGE_COUNT) {
          smsDetail = String(smsIndex) + " of " + String(SMS_MESSAGE_COUNT) + " sent";
          beginSMSMessage();
        } else {
          smsState = SMS_SENT;
          smsDetail = "both messages";
        }
      } else if (gsmBuffer.indexOf("ERROR") >= 0 || elapsed > 60000) {
        smsFail(smsIndex == 0
          ? "Network did not accept the SMS"
          : "Location SMS was not accepted (details SMS was sent)");
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

  // Example: ACCIDENT ALERT! Rider: Muhammad Irfan Bin Yusof. Motorcycle: Honda EX5,
  // plate JBR 1240. Impact: 3.4 g. Location in next SMS.
  String &details = smsMessages[0];
  details = "ACCIDENT ALERT! Rider: ";
  details += RIDER_NAME;
  details += ". Motorcycle: ";
  details += VEHICLE_MODEL;
  details += ", plate ";
  details += VEHICLE_PLATE;
  details += ". Impact: ";
  details += String(impactG, 1);
  details += " g. Location in next SMS.";

  // Example: Location of JBR 1240: Lat 3.139003, Long 101.686855.
  // Map: https://maps.google.com/?q=3.139003,101.686855
  String &location = smsMessages[1];
  location = "Location of ";
  location += VEHICLE_PLATE;

  if (gps.location.isValid()) {
    const String lat = String(gps.location.lat(), 6);
    const String lng = String(gps.location.lng(), 6);

    location += ": Lat ";
    location += lat;
    location += ", Long ";
    location += lng;
    location += ". Map: https://maps.google.com/?q=";
    location += lat;
    location += ",";
    location += lng;
  } else {
    location += ": no GPS fix at the time of the crash.";
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

// Prints the hotspot address to the Serial Monitor (115200 baud) whenever the connection changes.
void reportRouterConnection() {
  static bool wasConnected = false;
  const bool connected = WiFi.status() == WL_CONNECTED;

  if (connected && !wasConnected) {
    Serial.print("Connected to ");
    Serial.println(ROUTER_SSID);
    Serial.print("Dashboard on this network: http://");
    Serial.println(WiFi.localIP());
    Serial.print("Or: http://");
    Serial.print(MDNS_NAME);
    Serial.println(".local");
  } else if (!connected && wasConnected) {
    Serial.print("Lost connection to ");
    Serial.println(ROUTER_SSID);
  }

  wasConnected = connected;
}

void setup() {
  Serial.begin(115200);

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
  for (String &message : smsMessages) {
    message.reserve(170);
  }

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

  Serial.println();
  Serial.print("Wi-Fi network: ");
  Serial.println(AP_SSID);
  Serial.print("Dashboard on that network: http://");
  Serial.println(WiFi.softAPIP());

  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("Or: http://");
    Serial.print(MDNS_NAME);
    Serial.println(".local");
  } else {
    Serial.println("mDNS failed to start: use the IP address instead.");
  }

  if (routerConfigured) {
    Serial.print("Joining hotspot ");
    Serial.println(ROUTER_SSID);
  }

  server.on("/", HTTP_GET, []() {
    // Sent straight from flash, so the large page never has to be copied into RAM.
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
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

  if (ROUTER_SSID[0] != '\0') {
    reportRouterConnection();
  }

  server.handleClient();
  readGPS();
  delay(1);
}
