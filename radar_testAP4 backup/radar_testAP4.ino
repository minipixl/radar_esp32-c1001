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

// --- Zeit & Reset ---
const int RESET_HOUR = 12;
const int RESET_MINUTE = 0;
const char* NTP_SERVER = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

// --- Schlaf-Logik ---
const unsigned long MAX_AWAY_TIME_SEC = 1800; // 30 Min
const unsigned long MIN_SLEEP_TIME_SEC = 7200; // 2 Std

// --- NEU: Filter-Konfiguration ---
// Wenn Bewegung < diesem Wert UND Person da -> Wir werten als "Im Bett"
// Standard 150 ist sehr empfindlich. Bei Problemen auf 300 erhöhen.
const int MOVEMENT_THRESHOLD_SLEEP = 250; 
const int CONFIDENCE_COUNT = 5; // Sekunden zur Stabilisierung

bool apMode = false;
DFRobot_HumanDetection hu(&Serial1);
WebServer server(80);

// ---- Log Puffer ----
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

// ================== Daten Strukturen ==================
struct SensorData {
  int     presence;
  int     movement;
  int     movementParam;
  int     respirationRate;
  int     heartRate;
  uint8_t sleepState;
  unsigned long lastUpdate;
  bool    calculatedInBed; 
} sensorData = { -1, -1, 0, 0, 0, 0, 0, false };

struct DailySleepStats {
  unsigned long totalValidSleepSeconds;
  unsigned int  validSessionCount;
  unsigned long lastResetTime;
} dailyStats = { 0, 0, 0 };

struct SessionTracker {
  bool isActive;
  unsigned long startTime;
  unsigned long accumulatedSec;
  unsigned long awayStartMs;
  bool isAway;
} currentSession = { false, 0, 0, 0, false };

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

// Filter-State
int stableCount = 0;

String fmtDuration(uint32_t sec) {
  uint32_t h = sec / 3600;
  uint32_t m = (sec % 3600) / 60;
  char buf[12];
  sprintf(buf, "%uh %02umin", h, m);
  return String(buf);
}

// ================== CSS ==================
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
.nav{display:flex;gap:10px;margin-bottom:20px;}
.nav a{font-family:'Share Tech Mono',monospace;font-size:.75rem;letter-spacing:.1em;padding:6px 14px;border-radius:8px;border:1px solid var(--border);color:var(--accent);text-decoration:none;text-transform:uppercase;}
.nav a.active{background:var(--accent);color:#000;}
footer{margin-top:22px;font-size:.65rem;color:var(--muted);letter-spacing:.06em;font-family:'Share Tech Mono',monospace;}
</style>
)css";

// ================== Webserver Handler ==================

void handleRoot() {
  String inBedColor = sensorData.calculatedInBed ? "#2ecc71" : "#555";
  String inBedLabel = sensorData.calculatedInBed ? "In Bed (Calc)" : "Not in Bed";
  String modeStr = apMode ? "AP" : "STA";

  // FIX: Explizite Umwandlung in String() vor der Verkettung
  String html = String("<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta http-equiv='refresh' content='2'>") +
                String("<title>Radar – Fixed</title>") + 
                String(CSS) + 
                String("</head><body>");
  
  html += "<header><h1>Radar Sensor <span>Fixed Logic</span></h1></header>";
  html += "<div class='nav'><a href='/' class='active'>Live</a><a href='/sleep'>Night</a><a href='/daily'>Daily</a></div>";
  html += "<div class='grid'>";
  
  html += "<div class='card card-wide'><div class='label'>Calculated Status</div>";
  html += "<div class='value' style='color:" + inBedColor + "'>" + inBedLabel + "</div>";
  html += "<div class='unit'>Raw Presence: " + String(sensorData.presence) + " | Movement: " + String(sensorData.movementParam) + "</div>";
  html += "<div class='unit'>Threshold: " + String(MOVEMENT_THRESHOLD_SLEEP) + " | Stable: " + String(stableCount) + "/" + String(CONFIDENCE_COUNT) + "</div></div>";

  html += "<div class='card card-wide'><div class='label'>Current Session</div>";
  if(currentSession.isActive) {
    html += "<div class='value'>" + fmtDuration(currentSession.accumulatedSec) + "</div>";
    html += "<div class='unit'>Accumulated Sleep</div>";
    if(currentSession.isAway) {
       unsigned long awaySec = (millis() - currentSession.awayStartMs) / 1000;
       html += "<div class='unit' style='color:#f39c12'>Away for: " + String(awaySec) + "s</div>";
    }
  } else {
    html += "<div class='unit'>No active session</div>";
  }
  html += "</div></div>";
  html += "<footer>Mode: " + modeStr + "</footer></body></html>";
  
  server.send(200, "text/html", html);
}

void handleDaily() {
  String modeStr = apMode ? "AP" : "STA";
  // FIX: String() Wrapper
  String html = String("<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><meta http-equiv='refresh' content='5'>") +
                String("<title>Daily</title>") + 
                String(CSS) + 
                String("</head><body>");
  
  html += "<header><h1>Daily Summary</h1></header>";
  html += "<div class='nav'><a href='/'>Live</a><a href='/sleep'>Night</a><a href='/daily' class='active'>Daily</a></div>";
  html += "<div class='grid'>";
  html += "<div class='card card-wide'><div class='label'>Total Valid Sleep</div>";
  html += "<div class='value'>" + fmtDuration(dailyStats.totalValidSleepSeconds) + "</div>";
  html += "<div class='unit'>Sessions (>2h): " + String(dailyStats.validSessionCount) + "</div></div>";
  
  if(dailyStats.lastResetTime > 0) {
     unsigned long ageMin = (millis() - dailyStats.lastResetTime) / 60000;
     html += "<div class='card'><div class='label'>Last Reset</div>";
     html += "<div class='value' style='font-size:1.5rem'>" + String(ageMin) + "</div><div class='unit'>min ago</div></div>";
  }
  html += "</div>";
  html += "<footer>Mode: " + modeStr + "</footer></body></html>";
  server.send(200, "text/html", html);
}

void handleSleep() {
   String modeStr = apMode ? "AP" : "STA";
   // FIX: String() Wrapper
   String html = String("<!DOCTYPE html><html lang='en'><head><meta charset='UTF-8'><title>Sleep</title>") + 
                 String(CSS) + 
                 String("</head><body>");
   
   html += "<header><h1>Night Stats (Sensor Native)</h1></header>";
   html += "<div class='nav'><a href='/'>Live</a><a href='/sleep' class='active'>Night</a></div>";
   html += "<div class='grid'>";
   
   if(!nightStats.hasData) {
     html += "<div class='card card-wide'><div class='unit'>No data yet. Sleep until morning to get stats.</div></div>";
   } else {
     html += "<div class='card card-wide'><div class='label'>Score</div><div class='value'>" + String(nightStats.sleepScore) + "</div></div>";
     html += "<div class='card'><div class='label'>Turns</div><div class='value'>" + String(nightStats.turnCount) + "</div></div>";
     html += "<div class='card'><div class='label'>Apnea</div><div class='value'>" + String(nightStats.apneaEvents) + "</div></div>";
     html += "<div class='card'><div class='label'>Small Move</div><div class='value'>" + String(nightStats.smallBodyMove) + "%</div></div>";
     html += "<div class='card'><div class='label'>Large Move</div><div class='value'>" + String(nightStats.largeBodyMove) + "%</div></div>";
   }
   html += "</div><footer>Mode: " + modeStr + "</footer></body></html>";
   server.send(200, "text/html", html);
}

void handleJSON() {
  String json = "{";
  json += "\"presence\":" + String(sensorData.presence) + ",";
  json += "\"movement\":" + String(sensorData.movementParam) + ",";
  json += "\"calculatedInBed\":" + String(sensorData.calculatedInBed ? "true" : "false") + ",";
  json += "\"sessionAccumulated\":" + String(currentSession.accumulatedSec) + ",";
  json += "\"dailyTotal\":" + String(dailyStats.totalValidSleepSeconds);
  json += "}";
  server.send(200, "application/json", json);
}

// ================== Netzwerk & Zeit ==================
void startNetwork() {
  apMode = (digitalRead(MODE_PIN) == HIGH);
  if (apMode) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.println("[AP] Started: " + String(AP_SSID));
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("[WIFI] Connecting");
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) { 
      delay(500); 
      Serial.print("."); 
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[WIFI] Connected. IP: " + WiFi.localIP().toString());
      configTime(gmtOffset_sec, daylightOffset_sec, NTP_SERVER);
      Serial.println("[NTP] Time sync started");
    } else {
      Serial.println("\n[WIFI] Failed");
    }
  }
}

void checkDailyReset() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  
  if (timeinfo.tm_hour == RESET_HOUR && timeinfo.tm_min == RESET_MINUTE && timeinfo.tm_sec == 0) {
    logMsg("[RESET] Daily Reset at 12:00!");
    dailyStats.totalValidSleepSeconds = 0;
    dailyStats.validSessionCount = 0;
    dailyStats.lastResetTime = millis();
    currentSession.isActive = false;
    currentSession.accumulatedSec = 0;
    currentSession.isAway = false;
  }
}

// ================== DIE NEUE LOGIK ==================
void calculateSleepState() {
  int rawPresence = hu.smHumanData(hu.eHumanPresence);
  int rawMovement = hu.smHumanData(hu.eHumanMovingRange); // 0-1000
  
  sensorData.presence = rawPresence;
  sensorData.movementParam = rawMovement;

  // Logik: Wenn Person da UND Bewegung gering -> Wir definieren das als "Im Bett"
  bool shouldBeInBed = (rawPresence == 1 && rawMovement < MOVEMENT_THRESHOLD_SLEEP);

  // Hysterese / Filter gegen Rauschen
  if (shouldBeInBed) {
    stableCount++;
  } else {
    stableCount--;
  }
  
  // Clamp counter
  if (stableCount > CONFIDENCE_COUNT) stableCount = CONFIDENCE_COUNT;
  if (stableCount < 0) stableCount = 0;

  // Status wechseln nur wenn stabil
  if (stableCount == CONFIDENCE_COUNT && !sensorData.calculatedInBed) {
    sensorData.calculatedInBed = true;
    logMsg("[FILTER] Status -> IN BED (Mov: " + String(rawMovement) + ")");
  } else if (stableCount == 0 && sensorData.calculatedInBed) {
    sensorData.calculatedInBed = false;
    logMsg("[FILTER] Status -> NOT IN BED (Mov: " + String(rawMovement) + ")");
  }
}

void processSleepLogic() {
  bool inBed = sensorData.calculatedInBed;
  
  if (inBed) {
    if (!currentSession.isActive) {
      currentSession.isActive = true;
      currentSession.startTime = millis();
      currentSession.accumulatedSec = 0;
      currentSession.isAway = false;
      logMsg("[SESSION] Started");
    } else if (currentSession.isAway) {
      // Zurück gekommen
      unsigned long awaySec = (millis() - currentSession.awayStartMs) / 1000;
      if (awaySec <= MAX_AWAY_TIME_SEC) {
        currentSession.isAway = false;
        logMsg("[RETURN] Back in bed (Away: " + String(awaySec) + "s)");
      } else {
        // Zu lange weg -> alte Session beenden, neue starten
        endSession();
        currentSession.isActive = true; 
        currentSession.accumulatedSec = 0;
        currentSession.isAway = false;
        logMsg("[NEW] New session after timeout");
      }
    }
    if (!currentSession.isAway) currentSession.accumulatedSec++;
  } else {
    // Nicht im Bett
    if (currentSession.isActive && !currentSession.isAway) {
      currentSession.isAway = true;
      currentSession.awayStartMs = millis();
      logMsg("[AWAY] Left bed");
    } else if (currentSession.isActive && currentSession.isAway) {
      if ((millis() - currentSession.awayStartMs) / 1000 > MAX_AWAY_TIME_SEC) {
        endSession();
      }
    }
  }
}

void endSession() {
  unsigned long dur = currentSession.accumulatedSec;
  logMsg("[END] Session ended. Duration: " + fmtDuration(dur));
  if (dur >= MIN_SLEEP_TIME_SEC) {
    dailyStats.totalValidSleepSeconds += dur;
    dailyStats.validSessionCount++;
    logMsg("[OK] > 2h. Added to stats. Total: " + fmtDuration(dailyStats.totalValidSleepSeconds));
  } else {
    logMsg("[IGN] < 2h. Ignored.");
  }
  currentSession.isActive = false;
  currentSession.accumulatedSec = 0;
  currentSession.isAway = false;
}

// ================== Setup & Loop ==================
void setup() {
  Serial.begin(115200);
  pinMode(MODE_PIN, INPUT);
  delay(10);
  
#if defined(ESP32)
  // ESP32-C6 Pinout anpassen falls nötig, hier Standard für C6 SuperMini oft GPIO 6/7
  Serial1.begin(115200, SERIAL_8N1, 7, 6); 
#else
  Serial1.begin(115200);
#endif

  startNetwork();
  
  server.on("/", handleRoot);
  server.on("/daily", handleDaily);
  server.on("/sleep", handleSleep);
  server.on("/json", handleJSON);
  server.on("/log", HTTP_GET, [](){ 
    server.send(200, "text/plain", String(logBuf)); 
  });
  server.begin();
  Serial.println("[WEB] Server started");

  Serial.println("Init Sensor...");
  while (hu.begin() != 0) { 
    Serial.println("Sensor init error, retry...");
    delay(1000); 
  }
  Serial.println("Sensor init OK");
  
  hu.configWorkMode(hu.eSleepMode);
  hu.configLEDLight(hu.eHPLed, 0);
  
  dailyStats.lastResetTime = millis();
  logMsg("System started with Fixed Logic");
}

void loop() {
  server.handleClient();
  
  // WLAN reconnect falls STA Mode
  if (!apMode && WiFi.status() != WL_CONNECTED) {
     static unsigned long lastReconnect = 0;
     if(millis() - lastReconnect > 10000) {
        startNetwork();
        lastReconnect = millis();
     }
  }
  
  checkDailyReset();
  
  // 1. Rohdaten lesen und internen Status berechnen
  calculateSleepState();
  
  // 2. Session Logik anwenden
  processSleepLogic();
  
  // 3. Sensor Stats holen (für die Night-Page)
  sSleepStatistics stats = hu.getSleepStatistics();
  
  // Bewegungsdaten kommen aus dem Composite, nicht aus Statistics
  sSleepComposite comp = hu.getSleepComposite();

  if (stats.sleepQualityScore > 0) {
    nightStats.hasData = true;
    nightStats.sleepScore = stats.sleepQualityScore;
    nightStats.turnCount = stats.turnOverCount;
    nightStats.apneaEvents = stats.apneaEvents;
    
    // KORREKTUR: Daten aus 'comp' statt 'stats' lesen
    // Die Bibliothek speichert diese Werte oft als uint8_t Prozentwerte im Composite
    nightStats.largeBodyMove = comp.largeBodyMove; 
    nightStats.smallBodyMove = comp.minorBodyMove;   // In manchen Lib-Versionen auch 'smallBodyMove'
    
    nightStats.fetchedAt = millis();
  }

  delay(1000);
}