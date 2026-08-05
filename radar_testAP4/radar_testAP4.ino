#include "DFRobot_HumanDetection.h"
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include "arduino_secrets.h"

// ================== KONFIGURATION ==================
const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

const char* AP_SSID     = "RadarSensor-AP";
const char* AP_PASSWORD = "radar1234";

#define MODE_PIN 8

// --- Zeit & Reset Konfiguration ---
const int RESET_HOUR = 12;
const int RESET_MINUTE = 0;
const char* NTP_SERVER = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // CET (Winterzeit)
const int   daylightOffset_sec = 3600; // Sommerzeit-Offset (wird dynamisch gehandhabt oder manuell anpassen)

// --- Schlaf-Logik Konfiguration ---
const unsigned long MAX_AWAY_TIME_SEC = 1800; // 0.5 Stunden in Sekunden
const unsigned long MIN_SLEEP_TIME_SEC = 7200; // 2 Stunden in Sekunden

bool apMode = false;

DFRobot_HumanDetection hu(&Serial1);
WebServer server(80);

// ---- Remote-Debug-Log (Ringpuffer) ----
#define LOG_BUF_SIZE 20000
char logBuf[LOG_BUF_SIZE + 1];
size_t logBufLen = 0;

void logMsg(const String &msg) {
  Serial.println(msg);
  String line = "[" + String(millis() / 1000) + "s] " + msg + "\n";
  if (logBufLen + line.length() >= LOG_BUF_SIZE) {
    size_t keep = logBufLen / 2;
    memmove(logBuf, logBuf + (logBufLen - keep), keep);
    logBufLen = keep;
  }
  memcpy(logBuf + logBufLen, line.c_str(), line.length());
  logBufLen += line.length();
  logBuf[logBufLen] = '\0';
}

// ================== Sensor-Daten (Live) ==================
struct SensorData {
  int     presence;
  int     movement;
  int     movementParam;
  int     respirationRate;
  int     heartRate;
  uint8_t inBed;
  uint8_t sleepState;
  uint8_t breatheState;
  unsigned long lastUpdate;
} sensorData = { -1, -1, 0, 0, 0, 0, 0, 1, 0 };

// ================== Erweiterte Schlaf-Statistik (Tagesbasis) ==================
struct DailySleepStats {
  unsigned long totalValidSleepSeconds; // Nur Sessions > 2h
  unsigned int  validSessionCount;      // Anzahl gültiger Sessions
  unsigned long lastResetTime;          // Zeitpunkt des letzten Resets
} dailyStats = { 0, 0, 0 };

// Laufende Session Variablen
struct SessionTracker {
  bool isActive;                // Läuft gerade eine Session?
  unsigned long startTime;      // Wann startete die Session (erste Erkennung)
  unsigned long accumulatedSec; // Akkumulierte Schlafzeit INNERHALB der Session
  unsigned long awayStartMs;    // Wann wurde das Bett verlassen (für Unterbrechung)
  bool isAway;                  // Ist gerade weg?
} currentSession = { false, 0, 0, 0, false };

// Nacht-Statistik (vom Sensor)
struct NightStats {
  bool    hasData;
  uint32_t awakeDuration;
  uint32_t lightSleepDur;
  uint32_t deepSleepDur;
  uint8_t  sleepScore;
  uint8_t  avgRespiration;
  uint8_t  avgHeartRate;
  uint8_t  turnCount;
  uint8_t  apneaEvents;
  uint8_t  largeBodyMove;
  uint8_t  smallBodyMove;
  unsigned long fetchedAt;
} nightStats = { false, 0,0,0,0,0,0,0,0,0,0,0 };

// ================== Hilfsfunktionen ==================
String sleepStateLabel(uint8_t s) {
  switch (s) {
    case 0: return "Deep Sleep";
    case 1: return "Light Sleep";
    case 2: return "Awake";
    case 3: return "None";
    default: return "Unknown";
  }
}
String breatheStateLabel(uint8_t s) {
  switch (s) {
    case 1: return "Normal";
    case 2: return "Too fast";
    case 3: return "Too slow";
    case 4: return "None";
    default: return "–";
  }
}
String fmtDuration(uint32_t sec) {
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  char buf[12];
  sprintf(buf, "%uh %02umin", h, m);
  return String(buf);
}

// ================== CSS (Unverändert) ==================
const char* CSS = R"css(
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600&display=swap');
:root{--bg:#0d0d14;--surface:#12121e;--border:#1e2040;--accent:#00c8ff;--muted:#445;--text:#ccd6f6;--dim:#556;}
*{box-sizing:border-box;margin:0;padding:0;}
body{background:var(--bg);color:var(--text);font-family:'Exo 2',sans-serif;min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:24px 16px 32px;}
body::before{content:'';position:fixed;inset:0;background:repeating-linear-gradient(0deg,transparent,transparent 3px,rgba(0,200,255,.015) 3px,rgba(0,200,255,.015) 4px);pointer-events:none;z-index:999;}
header{display:flex;align-items:center;gap:10px;margin-bottom:28px;}
h1{font-family:'Share Tech Mono',monospace;font-size:1.25rem;letter-spacing:.12em;color:var(--accent);text-transform:uppercase;}
h1 span{color:var(--dim);font-size:.75rem;display:block;letter-spacing:.06em;}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;width:100%;max-width:420px;}
.card{background:var(--surface);border:1px solid var(--border);border-radius:14px;padding:18px 14px 14px;display:flex;flex-direction:column;align-items:center;gap:6px;position:relative;overflow:hidden;}
.card::after{content:'';position:absolute;top:0;left:0;right:0;height:2px;background:var(--accent);opacity:.18;}
.card-wide{grid-column:1/-1;}
.label{font-size:.65rem;letter-spacing:.12em;text-transform:uppercase;color:var(--dim);align-self:flex-start;}
.value{font-family:'Share Tech Mono',monospace;font-size:2.4rem;line-height:1;color:#eef;}
.unit{font-size:.65rem;color:var(--dim);}
.badge{display:inline-block;padding:3px 10px;border-radius:6px;font-size:.75rem;font-weight:600;letter-spacing:.04em;}
.dot{width:12px;height:12px;border-radius:50%;display:inline-block;margin-right:5px;vertical-align:middle;}
.nav{display:flex;gap:10px;margin-bottom:20px;}
.nav a{font-family:'Share Tech Mono',monospace;font-size:.75rem;letter-spacing:.1em;padding:6px 14px;border-radius:8px;border:1px solid var(--border);color:var(--accent);text-decoration:none;text-transform:uppercase;}
.nav a.active{background:var(--accent);color:#000;}
.stat-row{display:flex;justify-content:space-between;width:100%;padding:5px 0;border-bottom:1px solid var(--border);}
.stat-row:last-child{border-bottom:none;}
.stat-label{font-size:.75rem;color:var(--dim);}
.stat-val{font-family:'Share Tech Mono',monospace;font-size:.85rem;color:#eef;}
.score-ring{display:flex;flex-direction:column;align-items:center;gap:4px;}
.no-data{color:var(--muted);font-size:.85rem;text-align:center;padding:20px;}
footer{margin-top:22px;font-size:.65rem;color:var(--muted);letter-spacing:.06em;font-family:'Share Tech Mono',monospace;}
</style>
)css";

// ================== Webserver Handler (Unverändert bis auf neue Stats) ==================
void handleRoot() {
  unsigned long age = (millis() - sensorData.lastUpdate) / 1000;
  String presenceColor = sensorData.presence == 1 ? "#2ecc71" : (sensorData.presence == 0 ? "#95a5a6" : "#e74c3c");
  String presenceLabel = sensorData.presence == 1 ? "Present" : (sensorData.presence == 0 ? "Nobody" : "Error");
  String motionColor;
  String motionLabel;
  switch (sensorData.movement) {
    case 0:  motionLabel="None";   motionColor="#555";    break;
    case 1:  motionLabel="Still";  motionColor="#f39c12"; break;
    case 2:  motionLabel="Active"; motionColor="#2ecc71"; break;
    default: motionLabel="Error";  motionColor="#e74c3c"; break;
  }
  int gv = constrain(sensorData.movementParam, 0, 1000);
  int gvPct = gv / 10;
  float arcLen = 169.6f;
  float filled = arcLen * gvPct / 100.0f;
  float empty  = arcLen - filled;
  String gaugeColor = (gvPct < 30) ? "#2ecc71" : (gvPct < 70 ? "#f39c12" : "#e74c3c");
  String sleepColor;
  switch (sensorData.sleepState) {
    case 0: sleepColor="#f39c12"; break;
    case 1: sleepColor="#3498db"; break;
    case 2: sleepColor="#9b59b6"; break;
    case 3: sleepColor="#1abc9c"; break;
    default: sleepColor="#555";
  }
  String inBedColor = sensorData.inBed ? "#2ecc71" : "#555";
  String modeStr = apMode ? "AP " + String(AP_SSID) : "STA";

  String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='2'><title>Radar – Live</title>";
  html += CSS;
  html += "</head><body>";
  html += "<header><svg width='28' height='28' viewBox='0 0 28 28' fill='none'>";
  html += "<circle cx='14' cy='14' r='3' fill='#00c8ff'/>";
  html += "<circle cx='14' cy='14' r='7' stroke='#00c8ff' stroke-width='1.2' stroke-dasharray='3 2' opacity='.6'/>";
  html += "<circle cx='14' cy='14' r='12' stroke='#00c8ff' stroke-width='1' stroke-dasharray='2 3' opacity='.3'/>";
  html += "</svg><h1>Radar Sensor<span>ESP32-C6 / C1001</span></h1></header>";
  html += "<div class='nav'><a href='/' class='active'>Live</a><a href='/sleep'>Night Stats</a><a href='/daily'>Daily</a></div>";
  html += "<div class='grid'>";
  html += "<div class='card'><div class='label'>Presence</div>";
  html += "<div style='margin:4px 0'><span class='dot' style='background:" + presenceColor + ";box-shadow:0 0 6px " + presenceColor + "'></span>";
  html += "<span class='badge' style='background:" + presenceColor + "22;color:" + presenceColor + ";border:1px solid " + presenceColor + "44'>" + presenceLabel + "</span></div></div>";
  html += "<div class='card'><div class='label'>Motion</div>";
  html += "<div style='margin:4px 0'><span class='badge' style='background:" + motionColor + "22;color:" + motionColor + ";border:1px solid " + motionColor + "44'>" + motionLabel + "</span></div>";
  html += "<div class='unit'>movement state</div></div>";
  html += "<div class='card'><div class='label'>Respiration</div>";
  html += "<div class='value'>" + String(sensorData.respirationRate) + "</div>";
  html += "<div class='unit'>breaths / min</div>";
  html += "<div class='unit' style='color:#667'>" + breatheStateLabel(sensorData.breatheState) + "</div></div>";
  html += "<div class='card'><div class='label'>Heart Rate</div>";
  html += "<div class='value'>" + String(sensorData.heartRate) + "</div>";
  html += "<div class='unit'>bpm</div></div>";
  html += "<div class='card card-wide'><div class='label'>Sleep State</div>";
  html += "<div style='display:flex;gap:12px;margin:6px 0;align-items:center;flex-wrap:wrap'>";
  html += "<span class='badge' style='background:" + inBedColor + "22;color:" + inBedColor + ";border:1px solid " + inBedColor + "44'>";
  html += (sensorData.inBed ? "In bed" : "Not in bed") + String("</span>");
  html += "<span class='badge' style='background:" + sleepColor + "22;color:" + sleepColor + ";border:1px solid " + sleepColor + "44'>";
  html += sleepStateLabel(sensorData.sleepState) + "</span></div></div>";
  
  // Neue Karte: Aktuelle Session Info
  html += "<div class='card card-wide'><div class='label'>Current Session</div>";
  if(currentSession.isActive) {
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.2rem;color:#eef'>Running</div>";
    html += "<div class='unit'>Accumulated: " + fmtDuration(currentSession.accumulatedSec) + "</div>";
    if(currentSession.isAway) {
       unsigned long awaySec = (millis() - currentSession.awayStartMs) / 1000;
       html += "<div class='unit' style='color:#f39c12'>Away for: " + String(awaySec) + "s (Max: " + String(MAX_AWAY_TIME_SEC) + "s)</div>";
    } else {
       html += "<div class='unit' style='color:#2ecc71'>In Bed</div>";
    }
  } else {
    html += "<div class='unit'>No active session</div>";
  }
  html += "</div>";

  html += "<div class='card card-wide'><div class='label' style='align-self:flex-start;width:100%'>Movement Intensity</div>";
  html += "<div style='display:flex;align-items:center;gap:20px;width:100%;justify-content:center'>";
  html += "<svg width='120' height='70' viewBox='0 0 120 70' overflow='visible'>";
  html += "<path d='M6,60 A54,54 0 0,1 114,60' fill='none' stroke='#1e2040' stroke-width='10' stroke-linecap='round'/>";
  html += "<path d='M6,60 A54,54 0 0,1 114,60' fill='none' stroke='" + gaugeColor + "' stroke-width='10' stroke-linecap='round'";
  html += " stroke-dasharray='" + String(filled,1) + " " + String(empty+0.1f,1) + "'";
  html += " style='filter:drop-shadow(0 0 4px " + gaugeColor + ")'/>";
  html += "<text x='60' y='58' text-anchor='middle' font-family='Share Tech Mono,monospace' font-size='20' fill='#eef'>" + String(gvPct) + "%</text>";
  html += "<text x='60' y='68' text-anchor='middle' font-size='7' fill='#556'>0 — 100%</text>";
  html += "</svg>";
  html += "<div style='text-align:left'><div style='font-size:.65rem;color:#556;letter-spacing:.1em;text-transform:uppercase;margin-bottom:6px'>Raw value</div>";
  html += "<div style='font-family:Share Tech Mono,monospace;font-size:2rem;color:#eef'>" + String(gv) + "</div>";
  html += "<div style='font-size:.65rem;color:#445'>0 – 1000</div></div></div></div>";
  html += "</div>";
  html += "<footer>Last reading " + String(age) + "s ago &nbsp;|&nbsp; mode: " + modeStr + "</footer>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSleep() {
  String modeStr = apMode ? "AP " + String(AP_SSID) : "STA";
  String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='10'><title>Radar – Night Stats</title>";
  html += CSS;
  html += "</head><body>";
  html += "<header><svg width='28' height='28' viewBox='0 0 28 28' fill='none'>";
  html += "<circle cx='14' cy='14' r='3' fill='#00c8ff'/>";
  html += "<circle cx='14' cy='14' r='7' stroke='#00c8ff' stroke-width='1.2' stroke-dasharray='3 2' opacity='.6'/>";
  html += "<circle cx='14' cy='14' r='12' stroke='#00c8ff' stroke-width='1' stroke-dasharray='2 3' opacity='.3'/>";
  html += "</svg><h1>Night Statistics<span>ESP32-C6 / C1001</span></h1></header>";
  html += "<div class='nav'><a href='/'>Live</a><a href='/sleep' class='active'>Night Stats</a><a href='/daily'>Daily</a></div>";
  html += "<div class='grid'>";
  if (!nightStats.hasData) {
    html += "<div class='card card-wide'><div class='no-data'>No sleep data yet.<br>Data is recorded after the sensor detects that the sleep session has ended.</div></div>";
  } else {
    String scoreColor = nightStats.sleepScore >= 80 ? "#2ecc71" : (nightStats.sleepScore >= 60 ? "#f39c12" : "#e74c3c");
    String scoreLabel = nightStats.sleepScore >= 80 ? "Good" : (nightStats.sleepScore >= 60 ? "Fair" : "Poor");
    html += "<div class='card card-wide' style='flex-direction:row;justify-content:space-around;align-items:center'>";
    html += "<div class='score-ring'><svg width='90' height='90' viewBox='0 0 90 90'>";
    float scoreArc = 251.2f * nightStats.sleepScore / 100.0f;
    html += "<circle cx='45' cy='45' r='40' fill='none' stroke='#1e2040' stroke-width='8'/>";
    html += "<circle cx='45' cy='45' r='40' fill='none' stroke='" + scoreColor + "' stroke-width='8'";
    html += " stroke-dasharray='" + String(scoreArc,1) + " " + String(251.2f - scoreArc,1) + "'";
    html += " stroke-linecap='round' transform='rotate(-90 45 45)'";
    html += " style='filter:drop-shadow(0 0 5px " + scoreColor + ")'/>";
    html += "<text x='45' y='49' text-anchor='middle' font-family='Share Tech Mono,monospace' font-size='20' fill='#eef'>" + String(nightStats.sleepScore) + "</text>";
    html += "</svg><div class='unit' style='color:" + scoreColor + ";font-weight:600'>" + scoreLabel + " sleep</div></div>";
    html += "<div style='display:flex;flex-direction:column;gap:4px;font-family:Share Tech Mono,monospace'>";
    html += "<div style='font-size:.7rem;color:#556'>Ø Respiration</div><div style='color:#eef'>" + String(nightStats.avgRespiration) + " /min</div>";
    html += "<div style='font-size:.7rem;color:#556;margin-top:6px'>Ø Heart Rate</div><div style='color:#eef'>" + String(nightStats.avgHeartRate) + " bpm</div>";
    html += "</div></div>";
    html += "<div class='card'><div class='label'>Awake</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#f39c12'>" + fmtDuration(nightStats.awakeDuration) + "</div></div>";
    html += "<div class='card'><div class='label'>Light Sleep</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#3498db'>" + fmtDuration(nightStats.lightSleepDur) + "</div></div>";
    html += "<div class='card'><div class='label'>Deep Sleep</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#9b59b6'>" + fmtDuration(nightStats.deepSleepDur) + "</div></div>";
    uint32_t total = nightStats.awakeDuration + nightStats.lightSleepDur + nightStats.deepSleepDur;
    html += "<div class='card'><div class='label'>Total in bed</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#eef'>" + fmtDuration(total) + "</div></div>";
    html += "<div class='card card-wide'><div class='label' style='width:100%;margin-bottom:8px'>Details</div>";
    auto row = [&](String label, String val) {
      html += "<div class='stat-row'><span class='stat-label'>" + label + "</span><span class='stat-val'>" + val + "</span></div>";
    };
    row("Turns (position changes)", String(nightStats.turnCount));
    row("Apnea events",             String(nightStats.apneaEvents));
    row("Large body movement",      String(nightStats.largeBodyMove) + "%");
    row("Small body movement",      String(nightStats.smallBodyMove) + "%");
    html += "</div>";
  }
  html += "</div>";
  html += "<footer>Stats age: " + String(nightStats.hasData ? String((millis()-nightStats.fetchedAt)/60000) + " min ago" : String("n/a")) + " &nbsp;|&nbsp; mode: " + modeStr + "</footer>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// NEU: Daily Stats Page
void handleDaily() {
  String modeStr = apMode ? "AP " + String(AP_SSID) : "STA";
  String html = "<!DOCTYPE html><html lang='en'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'><title>Radar – Daily Stats</title>";
  html += CSS;
  html += "</head><body>";
  html += "<header><svg width='28' height='28' viewBox='0 0 28 28' fill='none'>";
  html += "<circle cx='14' cy='14' r='3' fill='#00c8ff'/>";
  html += "<circle cx='14' cy='14' r='7' stroke='#00c8ff' stroke-width='1.2' stroke-dasharray='3 2' opacity='.6'/>";
  html += "<circle cx='14' cy='14' r='12' stroke='#00c8ff' stroke-width='1' stroke-dasharray='2 3' opacity='.3'/>";
  html += "</svg><h1>Daily Summary<span>Since " + String(RESET_HOUR) + ":00</span></h1></header>";
  html += "<div class='nav'><a href='/'>Live</a><a href='/sleep'>Night Stats</a><a href='/daily' class='active'>Daily</a></div>";
  html += "<div class='grid'>";
  
  html += "<div class='card card-wide'><div class='label'>Total Valid Sleep</div>";
  html += "<div class='value'>" + fmtDuration(dailyStats.totalValidSleepSeconds) + "</div>";
  html += "<div class='unit'>Across " + String(dailyStats.validSessionCount) + " sessions (min 2h each)</div></div>";
  
  html += "<div class='card'><div class='label'>Sessions</div>";
  html += "<div class='value'>" + String(dailyStats.validSessionCount) + "</div></div>";
  
  html += "<div class='card'><div class='label'>Last Reset</div>";
  if(dailyStats.lastResetTime > 0) {
     unsigned long ageMin = (millis() - dailyStats.lastResetTime) / 60000;
     html += "<div class='value' style='font-size:1.5rem'>" + String(ageMin) + "</div>";
     html += "<div class='unit'>minutes ago</div>";
  } else {
     html += "<div class='unit'>Not yet reset</div>";
  }
  html += "</div>";
  
  html += "</div>";
  html += "<footer>Mode: " + modeStr + "</footer>";
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
  json += "\"inBed\":"            + String(sensorData.inBed)            + ",";
  json += "\"sleepState\":"       + String(sensorData.sleepState)       + ",";
  json += "\"sessionActive\":"    + String(currentSession.isActive ? "true" : "false") + ",";
  json += "\"sessionAccumulated\":" + String(currentSession.accumulatedSec) + ",";
  json += "\"dailyTotal\":"       + String(dailyStats.totalValidSleepSeconds);
  json += "}";
  server.send(200, "application/json", json);
}

// ================== WLAN / AP / NTP ==================
void startNetwork() {
  apMode = (digitalRead(MODE_PIN) == HIGH);
  if (apMode) {
    WiFi.mode(WIFI_AP);
    bool ok = (strlen(AP_PASSWORD) >= 8) ? WiFi.softAP(AP_SSID, AP_PASSWORD) : WiFi.softAP(AP_SSID);
    if (ok) {
      Serial.print("[AP]   SSID: "); Serial.println(AP_SSID);
      Serial.print("[AP]   IP:   "); Serial.println(WiFi.softAPIP());
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
      // NTP Starten wenn WLAN da ist
      configTime(gmtOffset_sec, daylightOffset_sec, NTP_SERVER);
      Serial.println("[NTP]  Zeit synchronisierung gestartet");
    } else {
      Serial.println("\n[WIFI] NICHT verbunden");
    }
  }
}

void reconnectWiFi() {
  if (apMode) return;
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
      delay(500);
    }
    if(WiFi.status() == WL_CONNECTED) {
       configTime(gmtOffset_sec, daylightOffset_sec, NTP_SERVER);
    }
  }
}

// ================== SCHLAF-LOGIK KERNEFUNKTIONEN ==================

void checkDailyReset() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return; // Zeit noch nicht synchronisiert
  }
  
  // Prüfen auf 12:00:00 (nur einmal pro Sekunde genau)
  if (timeinfo.tm_hour == RESET_HOUR && timeinfo.tm_min == RESET_MINUTE && timeinfo.tm_sec == 0) {
    logMsg("[RESET] Täglicher Reset um 12:00 Uhr!");
    dailyStats.totalValidSleepSeconds = 0;
    dailyStats.validSessionCount = 0;
    dailyStats.lastResetTime = millis();
    
    // Auch laufende Session zurücksetzen? Ja, sicherheitshalber, falls um genau 12 Uhr jemand schläft
    currentSession.isActive = false;
    currentSession.accumulatedSec = 0;
    currentSession.isAway = false;
    currentSession.awayStartMs = 0;
  }
}

void processSleepLogic(int inBed, int sleepState) {
  // inBed: 1 = im Bett, 0 = nicht im Bett
  
  if (inBed == 1) {
    // Person ist im Bett
    if (!currentSession.isActive) {
      // Neue Session starten
      currentSession.isActive = true;
      currentSession.startTime = millis();
      currentSession.accumulatedSec = 0;
      currentSession.isAway = false;
      currentSession.awayStartMs = 0;
      logMsg("[SESSION] Neue Session gestartet.");
    } else {
      // Session lief, Person kommt zurück
      if (currentSession.isAway) {
        unsigned long awayDurationMs = millis() - currentSession.awayStartMs;
        unsigned long awayDurationSec = awayDurationMs / 1000;
        
        if (awayDurationSec <= MAX_AWAY_TIME_SEC) {
          logMsg("[RETURN] Zurück im Bett nach " + String(awayDurationSec) + "s. Session wird fortgesetzt.");
          // Session weiterführen, Zeit wird unten addiert
        } else {
          // Zu lange weg -> Alte Session beenden (ohne Zählen wenn < 2h), neue starten
          logMsg("[TIMEOUT] Zu lange weg (" + String(awayDurationSec) + "s). Alte Session verworfen/beendet. Neue Session startet.");
          // Alte Session verwerfen (da wir hier eh neu starten)
          // Falls die alte Session > 2h war, müsste man sie vorher speichern. 
          // Aber da 'inBed' zwischendurch 0 war und wir erst jetzt merken dass es zu lang war,
          // hätten wir das eigentlich beim Übergang 1->0->1 merken müssen.
          // Korrektur: Das Ende wird normalerweise erkannt, wenn man aufsteht und NICHT zurückkommt.
          // Hier ist der Sonderfall: Man kommt zurück, aber zu spät.
          // Wir behandeln es als neuen Start.
        }
        currentSession.isAway = false;
        currentSession.awayStartMs = 0;
      }
    }
    
    // Zeit addieren (nur wenn nicht weg)
    if (!currentSession.isAway) {
      currentSession.accumulatedSec++;
    }
    
  } else {
    // Person nicht im Bett (inBed == 0)
    if (currentSession.isActive && !currentSession.isAway) {
      // Gerade erst aufgestanden
      currentSession.isAway = true;
      currentSession.awayStartMs = millis();
      logMsg("[AWAY] Person aufgestanden. Unterbrechungs-Timer läuft.");
    } else if (currentSession.isActive && currentSession.isAway) {
      // Ist schon weg, prüfen ob Timeout
      unsigned long currentAwaySec = (millis() - currentSession.awayStartMs) / 1000;
      if (currentAwaySec > MAX_AWAY_TIME_SEC) {
        // Session offiziell beenden
        endSession(false); // false = nicht zurückgekommen
      }
    }
  }
}

void endSession(bool cameBackTooLate) {
  unsigned long finalDuration = currentSession.accumulatedSec;
  logMsg("[END] Session beendet. Akkumulierte Zeit: " + fmtDuration(finalDuration));
  
  if (finalDuration >= MIN_SLEEP_TIME_SEC) {
    dailyStats.totalValidSleepSeconds += finalDuration;
    dailyStats.validSessionCount++;
    logMsg("[VALID] > 2h! Zur Tagesstatistik hinzugefügt. Total: " + fmtDuration(dailyStats.totalValidSleepSeconds));
  } else {
    logMsg("[IGNORE] < 2h. Nicht zur Tagesstatistik hinzugefügt.");
  }
  
  // Reset Tracker
  currentSession.isActive = false;
  currentSession.accumulatedSec = 0;
  currentSession.isAway = false;
  currentSession.awayStartMs = 0;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(MODE_PIN, INPUT);
  delay(10);

#if defined(ESP32)
  Serial1.begin(115200, SERIAL_8N1, /*rx=*/7, /*tx=*/6);
#else
  Serial1.begin(115200);
#endif

  startNetwork();

  server.on("/",      handleRoot);
  server.on("/sleep", handleSleep);
  server.on("/daily", handleDaily);
  server.on("/json",  handleJSON);
  server.on("/log",   HTTP_GET, [](){
    server.send(200, "text/plain", String(logBuf));
  });
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

  hu.configLEDLight(hu.eHPLed, 0);
  hu.sensorRet();
  
  // Init Zeitstempel falls kein Reset bis 12 Uhr passiert (z.B. bei Neustart)
  dailyStats.lastResetTime = millis(); 
}

// ================== LOOP ==================
void loop() {
  server.handleClient();
  reconnectWiFi();
  checkDailyReset(); // 1. Zeit & Reset prüfen

  // --- Live-Sensordaten ---
  sensorData.presence        = hu.smHumanData(hu.eHumanPresence);
  sensorData.movement        = hu.smHumanData(hu.eHumanMovement);
  sensorData.movementParam   = hu.smHumanData(hu.eHumanMovingRange);
  sensorData.respirationRate = hu.getBreatheValue();
  sensorData.heartRate       = hu.getHeartRate();
  sensorData.breatheState    = hu.getBreatheState();

  sSleepComposite comp = hu.getSleepComposite();
  sensorData.inBed = hu.smSleepData(hu.eInOrNotInBed);
  sensorData.sleepState = comp.sleepState;
  sensorData.lastUpdate = millis();

  // --- ANGEPASSTE LOGIK STATT ALTER SESSION-ERKENNUNG ---
  // Wir nutzen nur inBed (0/1) für die robuste Zeitmessung
  processSleepLogic(sensorData.inBed, sensorData.sleepState);

  // Logging nur bei relevanten Änderungen der eigenen Logik (nicht jedes Radar-Rauschen)
  // Das alte Logging wurde entfernt, um den Puffer zu schonen. 
  // Status sieht man im Webinterface unter "Current Session".

  // --- Sleep-Statistik (Vom Sensor) ---
  sSleepStatistics stats = hu.getSleepStatistics();
  if (stats.sleepQualityScore > 0) {
    // Wir nutzen hier weiterhin die Sensor-Daten für die "Night Stats" Seite,
    // aber verlassen uns für die "Daily Stats" auf unsere eigene Logik.
    uint32_t totalSleepSec = 0; // Platzhalter, da Sensor-Werte oft unzuverlässig sind bei kurzen Sessions
    
    nightStats.hasData        = true;
    nightStats.sleepScore     = stats.sleepQualityScore;
    nightStats.awakeDuration  = stats.wakeDuration;
    // Berechnung basierend auf Sensor-Prozenten (mit Vorsicht zu genießen)
    // Oder wir nutzen unsere letzte Session wenn sie gerade endete? 
    // Bleiben wir bei der Sensor-Logik für die "Night Stats" Seite wie im Original,
    // aber wissen, dass "Daily Stats" die verlässlichere Quelle ist.
    nightStats.lightSleepDur  = stats.shallowSleepPercentage; // % Werte hier direkt speichern zur Anzeige
    nightStats.deepSleepDur   = stats.deepSleepPercentage;
    nightStats.avgRespiration = stats.averageRespiration;
    nightStats.avgHeartRate   = stats.averageHeartbeat;
    nightStats.turnCount      = stats.turnOverCount;
    nightStats.apneaEvents    = stats.apneaEvents;
    nightStats.largeBodyMove  = comp.largeBodyMove;
    nightStats.smallBodyMove  = comp.minorBodyMove;
    nightStats.fetchedAt      = millis();
  }

  delay(1000); // 1 Sekunde Taktung für Zeitbasis
}