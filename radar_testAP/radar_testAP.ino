#include "DFRobot_HumanDetection.h"
#include <WiFi.h>
#include <WebServer.h>
#include "arduino_secrets.h"

// ================== KONFIGURATION ==================
const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

const char* AP_SSID     = "RadarSensor-AP";
const char* AP_PASSWORD = "radar1234";

#define MODE_PIN 8

bool apMode = false;

DFRobot_HumanDetection hu(&Serial1);
WebServer server(80);

// ================== Sensor-Daten (Live) ==================
struct SensorData {
  int     presence;
  int     movement;
  int     movementParam;
  int     respirationRate;
  int     heartRate;
  // Sleep-Composite-State (wird jede Sekunde aktualisiert)
  uint8_t inBed;          // 0=nicht im Bett, 1=im Bett
  uint8_t sleepState;     // 0=wach, 1=leichter Schlaf, 2=Tiefschlaf, 3=REM
  uint8_t breatheState;   // 1=normal, 2=zu schnell, 3=zu langsam, 4=keiner
  unsigned long lastUpdate;
} sensorData = { -1, -1, 0, 0, 0, 0, 0, 1, 0 };

// ================== Sleep-Statistik (Nacht) ==================
// Der Sensor liefert getSleepStatistics() erst wenn er die
// Schlafsession als beendet erkennt (Person steht auf).
// Wir speichern den letzten gültigen Stand.
struct NightStats {
  bool    hasData;          // wurde schon einmal Statistik empfangen?
  uint32_t awakeDuration;   // Wachzeit in Sekunden
  uint32_t lightSleepDur;   // Leichtschlaf in Sekunden
  uint32_t deepSleepDur;    // Tiefschlaf in Sekunden
  uint8_t  sleepScore;      // 0–100
  uint8_t  avgRespiration;  // Ø Atemfrequenz
  uint8_t  avgHeartRate;    // Ø Herzrate
  uint8_t  turnCount;       // Anzahl Lagenwechsel
  uint8_t  apneaEvents;     // Atemaussetzer
  uint8_t  largeBodyMove;   // % starke Bewegungen
  uint8_t  smallBodyMove;   // % leichte Bewegungen
  unsigned long fetchedAt;  // millis() zum Zeitpunkt des Abrufs
} nightStats = { false, 0,0,0,0,0,0,0,0,0,0,0 };

// ================== Hilfsfunktionen ==================
String sleepStateLabel(uint8_t s) {
  switch (s) {
    case 0: return "Awake";
    case 1: return "Light Sleep";
    case 2: return "Deep Sleep";
    case 3: return "REM";
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

// ================== CSS (gemeinsam für beide Seiten) ==================
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

// ================== /  (Live-Dashboard) ==================
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

  // Sleep state badge
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

  // Header
  html += "<header><svg width='28' height='28' viewBox='0 0 28 28' fill='none'>";
  html += "<circle cx='14' cy='14' r='3' fill='#00c8ff'/>";
  html += "<circle cx='14' cy='14' r='7' stroke='#00c8ff' stroke-width='1.2' stroke-dasharray='3 2' opacity='.6'/>";
  html += "<circle cx='14' cy='14' r='12' stroke='#00c8ff' stroke-width='1' stroke-dasharray='2 3' opacity='.3'/>";
  html += "</svg><h1>Radar Sensor<span>ESP32-C6 / DFRobot C1001</span></h1></header>";

  // Nav
  html += "<div class='nav'><a href='/' class='active'>Live</a><a href='/sleep'>Night Stats</a></div>";

  html += "<div class='grid'>";

  // Presence
  html += "<div class='card'><div class='label'>Presence</div>";
  html += "<div style='margin:4px 0'><span class='dot' style='background:" + presenceColor + ";box-shadow:0 0 6px " + presenceColor + "'></span>";
  html += "<span class='badge' style='background:" + presenceColor + "22;color:" + presenceColor + ";border:1px solid " + presenceColor + "44'>" + presenceLabel + "</span></div></div>";

  // Motion
  html += "<div class='card'><div class='label'>Motion</div>";
  html += "<div style='margin:4px 0'><span class='badge' style='background:" + motionColor + "22;color:" + motionColor + ";border:1px solid " + motionColor + "44'>" + motionLabel + "</span></div>";
  html += "<div class='unit'>movement state</div></div>";

  // Respiration
  html += "<div class='card'><div class='label'>Respiration</div>";
  html += "<div class='value'>" + String(sensorData.respirationRate) + "</div>";
  html += "<div class='unit'>breaths / min</div>";
  html += "<div class='unit' style='color:#667'>" + breatheStateLabel(sensorData.breatheState) + "</div></div>";

  // Heart Rate
  html += "<div class='card'><div class='label'>Heart Rate</div>";
  html += "<div class='value'>" + String(sensorData.heartRate) + "</div>";
  html += "<div class='unit'>bpm</div></div>";

  // In Bed + Sleep State (volle Breite)
  html += "<div class='card card-wide'><div class='label'>Sleep State</div>";
  html += "<div style='display:flex;gap:12px;margin:6px 0;align-items:center;flex-wrap:wrap'>";
  html += "<span class='badge' style='background:" + inBedColor + "22;color:" + inBedColor + ";border:1px solid " + inBedColor + "44'>";
  html += (sensorData.inBed ? "In bed" : "Not in bed") + String("</span>");
  html += "<span class='badge' style='background:" + sleepColor + "22;color:" + sleepColor + ";border:1px solid " + sleepColor + "44'>";
  html += sleepStateLabel(sensorData.sleepState) + "</span></div></div>";

  // Movement gauge (volle Breite)
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

  html += "</div>"; // .grid
  html += "<footer>Last reading " + String(age) + "s ago &nbsp;|&nbsp; mode: " + modeStr + "</footer>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ================== /sleep  (Nacht-Statistik) ==================
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
  html += "</svg><h1>Night Statistics<span>ESP32-C6 / DFRobot C1001</span></h1></header>";

  html += "<div class='nav'><a href='/'>Live</a><a href='/sleep' class='active'>Night Stats</a></div>";

  html += "<div class='grid'>";

  if (!nightStats.hasData) {
    html += "<div class='card card-wide'><div class='no-data'>No sleep data yet.<br>Data is recorded after the sensor detects that the sleep session has ended (person leaves bed).</div></div>";
  } else {
    unsigned long ageMin = (millis() - nightStats.fetchedAt) / 60000;

    // Score-Karte
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

    // Schlafdauer-Karten
    html += "<div class='card'><div class='label'>Awake</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#f39c12'>" + fmtDuration(nightStats.awakeDuration) + "</div></div>";

    html += "<div class='card'><div class='label'>Light Sleep</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#3498db'>" + fmtDuration(nightStats.lightSleepDur) + "</div></div>";

    html += "<div class='card'><div class='label'>Deep Sleep</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#9b59b6'>" + fmtDuration(nightStats.deepSleepDur) + "</div></div>";

    // Gesamt
    uint32_t total = nightStats.awakeDuration + nightStats.lightSleepDur + nightStats.deepSleepDur;
    html += "<div class='card'><div class='label'>Total in bed</div>";
    html += "<div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#eef'>" + fmtDuration(total) + "</div></div>";

    // Detailtabelle
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

  html += "</div>"; // .grid
  html += "<footer>Stats age: " + String(nightStats.hasData ? String((millis()-nightStats.fetchedAt)/60000) + " min ago" : String("n/a")) + " &nbsp;|&nbsp; mode: " + modeStr + "</footer>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ================== /json ==================
void handleJSON() {
  String json = "{";
  json += "\"presence\":"         + String(sensorData.presence)         + ",";
  json += "\"movement\":"         + String(sensorData.movement)         + ",";
  json += "\"movingRange\":"      + String(sensorData.movementParam)    + ",";
  json += "\"respirationRate\":"  + String(sensorData.respirationRate)  + ",";
  json += "\"heartRate\":"        + String(sensorData.heartRate)        + ",";
  json += "\"inBed\":"            + String(sensorData.inBed)            + ",";
  json += "\"sleepState\":"       + String(sensorData.sleepState)       + ",";
  json += "\"breatheState\":"     + String(sensorData.breatheState)     + ",";
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
  if (apMode) return;
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
  server.on("/json",  handleJSON);
  server.begin();
  Serial.println("[WEB]  Server gestartet (/, /sleep, /json)");

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

  // --- Live-Sensordaten ---
  sensorData.presence        = hu.smHumanData(hu.eHumanPresence);
  sensorData.movement        = hu.smHumanData(hu.eHumanMovement);
  sensorData.movementParam   = hu.smHumanData(hu.eHumanMovingRange);
  sensorData.respirationRate = hu.getBreatheValue();
  sensorData.heartRate       = hu.getHeartRate();
  sensorData.breatheState    = hu.getBreatheState();

  // Sleep-Composite: In-Bed-Status + Schlafphase
  sSleepComposite comp = hu.getSleepComposite();
  sensorData.inBed      = comp.inOrNotInBed;
  sensorData.sleepState = comp.sleepState;

  sensorData.lastUpdate = millis();

  // --- Sleep-Statistik (Nacht-Auswertung) ---
  // Sensor liefert Daten erst wenn er das Schlafende erkennt.
  // getSleepStatistics() gibt einen leeren Struct wenn noch keine Session vorbei.
  sSleepStatistics stats = hu.getSleepStatistics();
  // Nur speichern wenn sleepScore > 0 (=valide Daten vorhanden)
  if (stats.sleepQuality > 0) {
    nightStats.hasData       = true;
    nightStats.sleepScore    = stats.sleepQuality;
    nightStats.awakeDuration = stats.awakeDuration;
    nightStats.lightSleepDur = stats.lightSleepDuration;
    nightStats.deepSleepDur  = stats.deepSleepDuration;
    nightStats.avgRespiration= stats.averageRespiration;
    nightStats.avgHeartRate  = stats.averageHeartRate;
    nightStats.turnCount     = stats.turnOverNumber;
    nightStats.apneaEvents   = stats.apneaEvents;
    nightStats.largeBodyMove = stats.largeBodyMove;
    nightStats.smallBodyMove = stats.smallBodyMove;
    nightStats.fetchedAt     = millis();
    Serial.println("[SLEEP] Neue Nacht-Statistik empfangen!");
  }

  // --- Serial-Debug ---
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
  Serial.print("In bed:       "); Serial.println(sensorData.inBed);
  Serial.print("Sleep state:  "); Serial.println(sleepStateLabel(sensorData.sleepState));
  Serial.println("-----------------------");

  delay(1000);
}
