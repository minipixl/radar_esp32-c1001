#include "DFRobot_HumanDetection.h"
#include <WiFi.h>
#include <WebServer.h>
#include "arduino_secrets.h"

// ================== KONFIGURATION ==================
const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

// Access-Point-Konfiguration
const char* AP_SSID     = "RadarSensor-AP";
const char* AP_PASSWORD = "radar1234";   // mind. 8 Zeichen, oder "" für offen

// GPIO-Pin: HIGH (VCC) → AP-Modus, LOW (GND) → Station-Modus
#define MODE_PIN 8

bool apMode = false;

DFRobot_HumanDetection hu(&Serial1);
WebServer server(80);

// ================== Sensor-Daten ==================
struct SensorData {
  int     presence;        // 0 = niemand, 1 = jemand, -1 = Fehler
  int     movement;        // 0 = keine Bewegung, 1 = still, 2 = aktiv
  int     movementParam;   // Rohwert 0..1000 (smHumanData eHumanMovingRange)
  int     respirationRate;
  int     heartRate;
  unsigned long lastUpdate;
} sensorData = { -1, -1, 0, 0, 0, 0 };

// ================== Webserver-Handler ==================
void handleRoot() {
  unsigned long age = (millis() - sensorData.lastUpdate) / 1000;

  // Presence
  String presenceLabel, presenceColor, presenceDot;
  if (sensorData.presence == 1) {
    presenceLabel = "Someone present";
    presenceColor = "#2ecc71";
    presenceDot   = "#2ecc71";
  } else if (sensorData.presence == 0) {
    presenceLabel = "Nobody present";
    presenceColor = "#95a5a6";
    presenceDot   = "#555";
  } else {
    presenceLabel = "Read error";
    presenceColor = "#e74c3c";
    presenceDot   = "#e74c3c";
  }

  // Motion
  String motionLabel, motionColor;
  switch (sensorData.movement) {
    case 0:  motionLabel = "None";  motionColor = "#555";    break;
    case 1:  motionLabel = "Still"; motionColor = "#f39c12"; break;
    case 2:  motionLabel = "Active"; motionColor = "#2ecc71"; break;
    default: motionLabel = "Error"; motionColor = "#e74c3c"; break;
  }

  // Gauge-Wert clampen (eHumanMovingRange liefert 0..1000)
  int gv = sensorData.movementParam;
  if (gv < 0)    gv = 0;
  if (gv > 1000) gv = 1000;
  int gvPct = gv / 10;  // auf 0..100 skalieren für Anzeige
  // SVG-Arc für Gauge (Halbkreis, r=54, cx=60, cy=60)
  // Bogenlänge = Prozent × π × r  (Halbkreis-Umfang = π*54 ≈ 169.6)
  float arcLen   = 169.6f;
  float filled   = arcLen * gvPct / 100.0f;
  float empty    = arcLen - filled;
  // Farbe des Bogens je nach Wert
  String gaugeColor;
  if      (gvPct < 30) gaugeColor = "#2ecc71";
  else if (gvPct < 70) gaugeColor = "#f39c12";
  else              gaugeColor = "#e74c3c";

  String modeStr = apMode ? "AP " + String(AP_SSID) : "STA";

  String html = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="2">
<title>Radar Sensor</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600&display=swap');

  :root {
    --bg:      #0d0d14;
    --surface: #12121e;
    --border:  #1e2040;
    --accent:  #00c8ff;
    --muted:   #445;
    --text:    #ccd6f6;
    --dim:     #556;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Exo 2', sans-serif;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 24px 16px 32px;
  }
  /* subtle scanline overlay */
  body::before {
    content: '';
    position: fixed;
    inset: 0;
    background: repeating-linear-gradient(0deg,
      transparent, transparent 3px,
      rgba(0,200,255,.015) 3px, rgba(0,200,255,.015) 4px);
    pointer-events: none;
    z-index: 999;
  }
  header {
    display: flex;
    align-items: center;
    gap: 10px;
    margin-bottom: 28px;
  }
  header svg { flex-shrink: 0; }
  h1 {
    font-family: 'Share Tech Mono', monospace;
    font-size: 1.25rem;
    letter-spacing: .12em;
    color: var(--accent);
    text-transform: uppercase;
  }
  h1 span { color: var(--dim); font-size: .75rem; display: block; letter-spacing: .06em; }

  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 12px;
    width: 100%;
    max-width: 420px;
  }
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 14px;
    padding: 18px 14px 14px;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
    position: relative;
    overflow: hidden;
  }
  .card::after {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: var(--accent);
    opacity: .18;
  }
  .label {
    font-size: .65rem;
    letter-spacing: .12em;
    text-transform: uppercase;
    color: var(--dim);
    align-self: flex-start;
  }
  .value {
    font-family: 'Share Tech Mono', monospace;
    font-size: 2.4rem;
    line-height: 1;
    color: #eef;
  }
  .unit { font-size: .65rem; color: var(--dim); }
  .badge {
    display: inline-block;
    padding: 3px 10px;
    border-radius: 6px;
    font-size: .75rem;
    font-weight: 600;
    letter-spacing: .04em;
  }
  .dot {
    width: 12px; height: 12px;
    border-radius: 50%;
    display: inline-block;
    margin-right: 5px;
    vertical-align: middle;
  }

  /* Gauge card spans 2 columns */
  .card-gauge { grid-column: 1 / -1; }
  .gauge-wrap { display: flex; align-items: center; gap: 20px; width: 100%; justify-content: center; }
  .gauge-svg { overflow: visible; }

  footer {
    margin-top: 22px;
    font-size: .65rem;
    color: var(--muted);
    letter-spacing: .06em;
    font-family: 'Share Tech Mono', monospace;
  }
</style>
</head>
<body>
<header>
  <svg width="28" height="28" viewBox="0 0 28 28" fill="none">
    <circle cx="14" cy="14" r="3" fill="#00c8ff"/>
    <circle cx="14" cy="14" r="7" stroke="#00c8ff" stroke-width="1.2" stroke-dasharray="3 2" opacity=".6"/>
    <circle cx="14" cy="14" r="12" stroke="#00c8ff" stroke-width="1" stroke-dasharray="2 3" opacity=".3"/>
  </svg>
  <h1>Radar Sensor<span>ESP32-C6 / DFRobot C1001</span></h1>
</header>

<div class="grid">
)rawhtml";

  // ---- Card: Presence ----
  html += "<div class='card'>";
  html += "<div class='label'>Presence</div>";
  html += "<div style='margin:4px 0'><span class='dot' style='background:" + presenceDot + ";box-shadow:0 0 6px " + presenceDot + "'></span>";
  html += "<span class='badge' style='background:" + presenceColor + "22;color:" + presenceColor + ";border:1px solid " + presenceColor + "44'>" + presenceLabel + "</span></div>";
  html += "</div>";

  // ---- Card: Motion ----
  html += "<div class='card'>";
  html += "<div class='label'>Motion</div>";
  html += "<div style='margin:4px 0'><span class='badge' style='background:" + motionColor + "22;color:" + motionColor + ";border:1px solid " + motionColor + "44'>" + motionLabel + "</span></div>";
  html += "<div class='unit'>movement state</div>";
  html += "</div>";

  // ---- Card: Respiration ----
  html += "<div class='card'>";
  html += "<div class='label'>Respiration</div>";
  html += "<div class='value'>" + String(sensorData.respirationRate) + "</div>";
  html += "<div class='unit'>breaths / min</div>";
  html += "</div>";

  // ---- Card: Heart Rate ----
  html += "<div class='card'>";
  html += "<div class='label'>Heart Rate</div>";
  html += "<div class='value'>" + String(sensorData.heartRate) + "</div>";
  html += "<div class='unit'>bpm</div>";
  html += "</div>";

  // ---- Card: Movement Gauge (full width) ----
  // SVG half-circle gauge
  // Halbkreis: path von (6,60) über (60,6) nach (114,60), r=54
  // stroke-dasharray: filled empty, offset=-arcLen so es von links läuft
  html += "<div class='card card-gauge'>";
  html += "<div class='label' style='align-self:flex-start;width:100%'>Movement Intensity</div>";
  html += "<div class='gauge-wrap'>";
  html += "<svg class='gauge-svg' width='120' height='70' viewBox='0 0 120 70'>";
  // Hintergrund-Bogen
  html += "<path d='M6,60 A54,54 0 0,1 114,60' fill='none' stroke='#1e2040' stroke-width='10' stroke-linecap='round'/>";
  // Wert-Bogen
  html += "<path d='M6,60 A54,54 0 0,1 114,60' fill='none' stroke='" + gaugeColor + "' stroke-width='10' stroke-linecap='round'";
  html += " stroke-dasharray='" + String(filled, 1) + " " + String(empty + 0.1f, 1) + "'";
  html += " style='filter:drop-shadow(0 0 4px " + gaugeColor + ")'/>";
  // Wert-Text
  html += "<text x='60' y='58' text-anchor='middle' font-family='Share Tech Mono,monospace' font-size='20' fill='#eef'>" + String(gvPct) + "%</text>";
  html += "<text x='60' y='68' text-anchor='middle' font-family='Exo 2,sans-serif' font-size='7' fill='#556'>0 — 100%</text>";
  html += "</svg>";
  html += "<div style='text-align:left'>";
  html += "<div style='font-size:.65rem;color:#556;letter-spacing:.1em;text-transform:uppercase;margin-bottom:6px'>Raw value</div>";
  html += "<div style='font-family:Share Tech Mono,monospace;font-size:2rem;color:#eef'>" + String(gv) + "</div>";
  html += "<div style='font-size:.65rem;color:#445'>0 – 1000</div>";
  html += "</div>";
  html += "</div></div>";

  html += "</div>"; // .grid

  html += "<footer>Last reading " + String(age) + "s ago &nbsp;|&nbsp; mode: " + modeStr + "</footer>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleJSON() {
  String json = "{";
  json += "\"presence\":"         + String(sensorData.presence)         + ",";
  json += "\"movement\":"         + String(sensorData.movement)         + ",";
  json += "\"movingRange\":"      + String(sensorData.movementParam)    + ",";
  json += "\"respirationRate\":"  + String(sensorData.respirationRate)  + ",";
  json += "\"heartRate\":"        + String(sensorData.heartRate)        + ",";
  json += "\"ageMs\":"            + String(millis() - sensorData.lastUpdate);
  json += "}";
  server.send(200, "application/json", json);
}

// ================== WLAN / AP ==================
void startNetwork() {
  apMode = (digitalRead(MODE_PIN) == HIGH);

  if (apMode) {
    WiFi.mode(WIFI_AP);
    bool ok = (strlen(AP_PASSWORD) >= 8)
              ? WiFi.softAP(AP_SSID, AP_PASSWORD)
              : WiFi.softAP(AP_SSID);
    if (ok) {
      Serial.print("[AP]   SSID: "); Serial.println(AP_SSID);
      Serial.print("[AP]   IP:   "); Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("[AP]   Fehler beim Starten des Access Points!");
    }
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("[WIFI] Verbinde");
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("\n[WIFI] Verbunden, IP: "); Serial.println(WiFi.localIP());
    } else {
      Serial.println("\n[WIFI] NICHT verbunden");
    }
  }
}

void reconnectWiFi() {
  if (apMode) return;  // Im AP-Modus kein Reconnect nötig
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("[WIFI] Wiederverbinden...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(500); Serial.print(".");
    }
    Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " Fehler");
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  // MODE_PIN lesen (interner Pull-Down: HIGH = VCC = AP, LOW = GND = STA)
  pinMode(MODE_PIN, INPUT);
  delay(10); // Pin einschwingen lassen

#if defined(ESP32)
  Serial1.begin(115200, SERIAL_8N1, /*rx=*/7, /*tx=*/6);
#else
  Serial1.begin(115200);
#endif

  startNetwork();

  server.on("/",     handleRoot);
  server.on("/json", handleJSON);
  server.begin();
  Serial.println("[WEB]  Server gestartet");

  Serial.println("Start initialization");
  while (hu.begin() != 0) {
    Serial.println("init error!!!");
    delay(1000);
  }
  Serial.println("Initialization successful");

  Serial.println("Start switching work mode");
  while (hu.configWorkMode(hu.eSleepMode) != 0) {
    Serial.println("error!!!");
    delay(1000);
  }
  Serial.println("Work mode switch successful");

  Serial.print("Current work mode: ");
  switch (hu.getWorkMode()) {
    case 1:  Serial.println("Fall detection mode");  break;
    case 2:  Serial.println("Sleep detection mode"); break;
    default: Serial.println("Read error");
  }

  hu.configLEDLight(hu.eHPLed, 1);
  hu.sensorRet();

  Serial.print("HP LED status: ");
  switch (hu.getLEDLightState(hu.eHPLed)) {
    case 0:  Serial.println("Off"); break;
    case 1:  Serial.println("On");  break;
    default: Serial.println("Read error");
  }

  Serial.println();
}

// ================== LOOP ==================
void loop() {
  server.handleClient();
  reconnectWiFi();

  sensorData.presence       = hu.smHumanData(hu.eHumanPresence);
  sensorData.movement       = hu.smHumanData(hu.eHumanMovement);
  sensorData.movementParam  = hu.smHumanData(hu.eHumanMovingRange);
  sensorData.respirationRate = hu.getBreatheValue();
  sensorData.heartRate      = hu.getHeartRate();
  sensorData.lastUpdate     = millis();

  Serial.print("Presence:     ");
  switch (sensorData.presence) {
    case 0:  Serial.println("Nobody");  break;
    case 1:  Serial.println("Someone"); break;
    default: Serial.println("Error");
  }
  Serial.print("Movement:     "); Serial.println(sensorData.movement);
  Serial.print("Moving range: "); Serial.println(sensorData.movementParam);
  Serial.print("Respiration:  "); Serial.println(sensorData.respirationRate);
  Serial.print("Heart rate:   "); Serial.println(sensorData.heartRate);
  Serial.println("-----------------------");

  delay(1000);
}