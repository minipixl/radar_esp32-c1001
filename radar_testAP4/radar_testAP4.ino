/*********************************************************************
 * Radar-Schlafüberwachung mit ESP32-C6 SuperMini + DFRobot C1001 + MQTT
 * Umsetzung der Funktionalen Spezifikation funct_spec.md
 *
 * Benötigte Libraries (über Bibliotheksverwalter bzw. GitHub):
 *   - DFRobot_HumanDetection   (DFRobot, github.com/DFRobot/DFRobot_HumanDetection)
 *   - PubSubClient             (Nick O'Leary, Bibliotheksverwalter)
 *   - WiFi                     (im ESP32-Board-Package enthalten)
 *   - WebServer                (im ESP32-Board-Package enthalten)
 *
 * WICHTIG: Die Feldnamen von sSleepComposite / smSleepData()-Enums
 * unterscheiden sich zwischen Versionen der DFRobot-Library (siehe
 * eigene Erfahrung aus früheren Projekten). Die hier verwendeten Namen
 * (shallowSleepPercentage, deepSleepPercentage, wakeDuration,
 * averageHeartbeat, turnOverCount, eWakeDuration, eDeepSleepDuration)
 * entsprechen dem zuletzt bei dir funktionierenden Stand. Bitte gegen
 * die tatsächlich installierte DFRobot_HumanDetection.h/.cpp prüfen,
 * falls der Compiler "no member named ..." meldet.
 *********************************************************************/

#include "DFRobot_HumanDetection.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <WebServer.h>
#include "arduino_secrets.h"  // SECRET_SSID, SECRET_PASS, SECRET_AP_SSID, SECRET_AP_PASSWORD

// ==================== KONFIGURATION ====================

// --- WLAN / AP Fallback (Abschnitt 2) ---
const char* WLAN_SSID     = SECRET_SSID;
const char* WLAN_PASSWORD = SECRET_PASS;
const char* AP_SSID       = SECRET_AP_SSID;      // "RadarSensor-AP"
const char* AP_PASSWORD   = SECRET_AP_PASSWORD;  // "radar1234"
const uint32_t WIFI_RETRY_INTERVAL_MS = 60UL * 1000UL; // alle 60s Reconnect-Versuch

// --- MQTT / InfluxDB-Pipeline (Abschnitt 3) ---
const char* MQTT_BROKER      = "192.168.1.139";  // IP des Raspi
const uint16_t MQTT_PORT     = 1883;
const char* MQTT_USER        = "";
const char* MQTT_PASS        = "";
const char* MQTT_CLIENT_ID   = "esp32c6-radar-sleep";
const char* MQTT_TOPIC       = "home/radar/sensor/data";   // jede Sekunde
const char* MQTT_STATS_TOPIC = "home/radar/sleep/stats";   // pro abgeschlossener Session
const char* MQTT_LOG_TOPIC   = "home/radar/log";

// --- Session-Logik (Abschnitt 4) ---
const uint32_t INTERRUPTION_WINDOW_MS   = 5UL * 60UL * 1000UL;  // 5 Min. Toleranz für kurzes Aufstehen
const uint32_t MIN_DURATION_FOR_SESSION = 20UL * 60UL * 1000UL; // Mindestlänge, damit eine Session zählt (20 Min.)
const uint32_t SENSOR_READ_INTERVAL_MS  = 1000UL;               // 1x pro Sekunde messen/publishen

// --- Ringpuffer für /log ---
const uint8_t LOG_BUFFER_SIZE = 40;

// --- Web-Stylesheet ---
// Hinweis: "SS" ist auf dem ESP32-C6 SuperMini bereits als SPI-Pin (uint8_t)
// in pins_arduino.h vordefiniert -> anderer Name nötig.
const char* RADAR_CSS =
  "body{background:#0b0e14;color:#dfe6f0;font-family:'Segoe UI',sans-serif;margin:0;padding:0}"
  "header{display:flex;align-items:center;gap:10px;padding:16px 20px;border-bottom:1px solid #1c2431}"
  "header h1{font-size:1.1rem;margin:0;display:flex;flex-direction:column}"
  "header h1 span{font-size:.7rem;color:#7a8699;font-weight:400}"
  ".nav{display:flex;gap:14px;padding:10px 20px;border-bottom:1px solid #1c2431}"
  ".nav a{color:#7a8699;text-decoration:none;font-size:.85rem;padding:4px 8px;border-radius:6px}"
  ".nav a.active{color:#00c8ff;background:#00c8ff1a}"
  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px;padding:20px}"
  ".card{background:#11151f;border:1px solid #1c2431;border-radius:10px;padding:14px}"
  ".label{font-size:.75rem;color:#7a8699;text-transform:uppercase;letter-spacing:.05em}"
  ".dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px;vertical-align:middle}"
  ".badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.85rem;vertical-align:middle}"
  ".bar{height:10px;border-radius:6px;background:#1c2431;overflow:hidden;display:flex;margin-top:6px}"
  ".seg-deep{background:#2ecc71}.seg-light{background:#3498db}.seg-awake{background:#e74c3c}";

// ==================== DATENSTRUKTUREN ====================

struct SensorData {
  int presence        = -1;
  int movement        = -1;
  int movementParam   = -1;
  int breatheState    = -1;
  float respirationRate = 0;
  float heartRate       = 0;
  int inBed            = -1;
  int sleepState        = -1; // 0=Deep,1=Light,2=Awake,3=None
  unsigned long lastUpdate = 0;
};

struct NightStats {
  unsigned long sessionStartMs   = 0;
  unsigned long sessionEndMs     = 0;
  uint32_t sessionDurationSec    = 0;
  uint32_t wakeDurationSec       = 0;
  uint32_t deepSleepDurationSec  = 0;
  uint32_t lightSleepDurationSec = 0;
  uint8_t  lightSleepPercentage  = 0;
  uint8_t  deepSleepPercentage   = 0;
  uint8_t  sleepScore            = 0;
  float    avgRespiration        = 0;
  float    avgHeartRate          = 0;
  uint16_t turnCount             = 0;
  uint16_t apneaEvents           = 0;
  uint16_t largeBodyMove         = 0;
  uint16_t smallBodyMove         = 0;
  bool     valid                 = false; // true sobald mind. eine Session abgeschlossen wurde
};

enum SessionState {
  OUT_OF_BED,   // niemand im Bett, keine laufende Session
  IN_BED,       // Session läuft, aktuell im Bett
  GRACE_EXIT    // kurzzeitig aus dem Bett (< INTERRUPTION_WINDOW_MS), Session pausiert aber noch aktiv
};

// ==================== GLOBALE OBJEKTE ====================

DFRobot_HumanDetection hu(&Serial1);
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

SensorData sensorData;
NightStats nightStats;         // letzte abgeschlossene Session
SessionState sessionState = OUT_OF_BED;
unsigned long sessionStartMs = 0;
unsigned long graceStartMs   = 0;

String logBuffer[LOG_BUFFER_SIZE];
uint8_t logIndex = 0;
bool logFull = false;

bool apMode = false;
bool sensorReady = false;
unsigned long lastWifiRetry = 0;
unsigned long lastSensorRead = 0;
unsigned long lastSensorInitAttempt = 0;
const uint32_t SENSOR_INIT_RETRY_MS = 5000; // alle 5s erneut versuchen, falls Sensor nicht antwortet

// ==================== HILFSFUNKTIONEN ====================

const char* sleepStateLabel(int state) {
  switch (state) {
    case 0: return "Deep Sleep";
    case 1: return "Light Sleep";
    case 2: return "Awake";
    case 3: return "None";
    default: return "Unknown";
  }
}

void logMsg(const String& msg) {
  String line = "[" + String(millis() / 1000) + "s] " + msg;
  Serial.println(line);
  logBuffer[logIndex] = line;
  logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
  if (logIndex == 0) logFull = true;
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_LOG_TOPIC, line.c_str());
  }
}

// ==================== WLAN / AP FALLBACK ====================

void startNetwork() {
  Serial.print("[WIFI] Verbinde mit ");
  Serial.println(WLAN_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    Serial.println();
    Serial.print("[WIFI] Verbunden, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("[WIFI] Kein WLAN gefunden -> starte Access Point Fallback");
    apMode = true;
    WiFi.mode(WIFI_AP);
    bool ok = (strlen(AP_PASSWORD) >= 8)
                ? WiFi.softAP(AP_SSID, AP_PASSWORD)
                : WiFi.softAP(AP_SSID);
    if (ok) {
      Serial.print("[AP]   SSID: ");
      Serial.println(AP_SSID);
      Serial.print("[AP]   IP:   ");
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("[AP]   Fehler beim Starten des Access Points!");
    }
  }
  lastWifiRetry = millis();
}

// Versucht alle 60s erneut, zurück ins Heim-WLAN zu wechseln bzw. die Verbindung
// wiederherzustellen (Abschnitt 2: "Wiederverbindung: Versuch alle 60 Sekunden").
void reconnectWiFi() {
  if (millis() - lastWifiRetry < WIFI_RETRY_INTERVAL_MS) return;
  lastWifiRetry = millis();

  if (!apMode && WiFi.status() == WL_CONNECTED) return; // alles gut

  Serial.println("[WIFI] Reconnect-Versuch...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    Serial.println(" OK, zurück im Heim-WLAN");
  } else if (!apMode) {
    // War vorher im STA-Modus und hat es nicht geschafft -> auf AP umschalten
    Serial.println(" Fehler, wechsle in AP-Modus");
    apMode = true;
    WiFi.mode(WIFI_AP);
    (strlen(AP_PASSWORD) >= 8) ? WiFi.softAP(AP_SSID, AP_PASSWORD) : WiFi.softAP(AP_SSID);
  } else {
    Serial.println(" weiterhin kein WLAN, bleibe im AP-Modus");
  }
}

// ==================== MQTT ====================

void reconnectMQTT() {
  if (apMode) return; // im AP-Modus kein Zugriff auf den Pi
  if (mqttClient.connected()) return;

  Serial.print("[MQTT] Verbinde...");
  bool ok = (strlen(MQTT_USER) > 0)
              ? mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)
              : mqttClient.connect(MQTT_CLIENT_ID);
  if (ok) {
    Serial.println(" OK");
  } else {
    Serial.print(" Fehler, rc=");
    Serial.println(mqttClient.state());
  }
}

void publishRawData() {
  if (!mqttClient.connected()) return;

  String payload = "{";
  payload += "\"presence\":"     + String(sensorData.presence)        + ",";
  payload += "\"movement\":"     + String(sensorData.movement)        + ",";
  payload += "\"inBed\":"        + String(sensorData.inBed)           + ",";
  payload += "\"sleepState\":"   + String(sensorData.sleepState)      + ",";
  payload += "\"breatheState\":" + String(sensorData.breatheState)    + ",";
  payload += "\"respiration\":"  + String(sensorData.respirationRate) + ",";
  payload += "\"heartRate\":"    + String(sensorData.heartRate)       + ",";
  payload += "\"millis\":"       + String(millis());
  payload += "}";

  mqttClient.publish(MQTT_TOPIC, payload.c_str());
}

void publishNightStats() {
  if (!mqttClient.connected() || !nightStats.valid) return;

  String payload = "{";
  payload += "\"sessionDurationSec\":"    + String(nightStats.sessionDurationSec)    + ",";
  payload += "\"wakeDurationSec\":"       + String(nightStats.wakeDurationSec)       + ",";
  payload += "\"deepSleepDurationSec\":"  + String(nightStats.deepSleepDurationSec)  + ",";
  payload += "\"lightSleepDurationSec\":" + String(nightStats.lightSleepDurationSec) + ",";
  payload += "\"lightSleepPercentage\":"  + String(nightStats.lightSleepPercentage)  + ",";
  payload += "\"deepSleepPercentage\":"   + String(nightStats.deepSleepPercentage)   + ",";
  payload += "\"sleepScore\":"            + String(nightStats.sleepScore)            + ",";
  payload += "\"avgRespiration\":"        + String(nightStats.avgRespiration)        + ",";
  payload += "\"avgHeartRate\":"          + String(nightStats.avgHeartRate)          + ",";
  payload += "\"turnCount\":"             + String(nightStats.turnCount)             + ",";
  payload += "\"apneaEvents\":"           + String(nightStats.apneaEvents)           + ",";
  payload += "\"largeBodyMove\":"         + String(nightStats.largeBodyMove)         + ",";
  payload += "\"smallBodyMove\":"         + String(nightStats.smallBodyMove)         + ",";
  payload += "\"sessionEndMillis\":"      + String(nightStats.sessionEndMs);
  payload += "}";

  mqttClient.publish(MQTT_STATS_TOPIC, payload.c_str(), true); // retained
}

// ==================== SESSION STATE MACHINE (Abschnitt 4) ====================
//
// Zustandsübergänge:
//   OUT_OF_BED  --inBed==1-->  IN_BED           (neue Session startet)
//   IN_BED      --inBed==0-->  GRACE_EXIT        (Session pausiert, Toleranzfenster startet)
//   GRACE_EXIT  --inBed==1-->  IN_BED            (Session läuft weiter, kein Abbruch)
//   GRACE_EXIT  --Timeout(5min)-->  OUT_OF_BED   (Session wird abgeschlossen & ausgewertet)
//
// Dadurch werden kurze Aufstehphasen (<5 Min, z.B. Toilette) nicht mehr als
// Sessionende fehlinterpretiert.

void runSessionStateMachine() {
  bool inBedNow = (sensorData.inBed == 1);

  switch (sessionState) {

    case OUT_OF_BED:
      if (inBedNow) {
        sessionStartMs = millis();
        sessionState = IN_BED;
        logMsg("[SESSION] Bett betreten - neue Session gestartet");
      }
      break;

    case IN_BED:
      if (!inBedNow) {
        graceStartMs = millis();
        sessionState = GRACE_EXIT;
        logMsg("[SESSION] Bett verlassen - Toleranzfenster (5 Min) gestartet");
      }
      break;

    case GRACE_EXIT:
      if (inBedNow) {
        sessionState = IN_BED;
        logMsg("[SESSION] Zurück im Bett - Session laeuft weiter");
      } else if (millis() - graceStartMs > INTERRUPTION_WINDOW_MS) {
        // Toleranzfenster abgelaufen -> Session gilt als beendet
        finalizeSession();
        sessionState = OUT_OF_BED;
      }
      break;
  }
}

void finalizeSession() {
  uint32_t sessionDurationMs = millis() - sessionStartMs;
  uint32_t sessionDurationSec = sessionDurationMs / 1000;

  if (sessionDurationSec < (MIN_DURATION_FOR_SESSION / 1000)) {
    logMsg("[SESSION] Zu kurz (" + String(sessionDurationSec) + "s), wird verworfen");
    return;
  }

  // Laut Spec: eWakeDuration / eDeepSleepDuration (smSleepData) statt
  // sSleepStatistics.sleepTime (kumulativer Zähler) nutzen. Der Sensor
  // liefert diese beiden Werte in MINUTEN (siehe DFRobot_HumanDetection.cpp,
  // case eWakeDuration / eDeepSleepDuration).
  uint32_t wakeDurationMin      = hu.smSleepData(hu.eWakeDuration);
  uint32_t deepSleepDurationMin = hu.smSleepData(hu.eDeepSleepDuration);

  // sleepQualityScore, shallowSleepPercentage, deepSleepPercentage und
  // turnOverCount liegen NICHT in sSleepComposite (getSleepComposite()),
  // sondern in sSleepStatistics (getSleepStatistics()).
  sSleepStatistics stats = hu.getSleepStatistics();
  // largeBodyMove/minorBodyMove gibt es nur im Composite-Struct.
  sSleepComposite comp = hu.getSleepComposite();

  nightStats.sessionStartMs      = sessionStartMs;
  nightStats.sessionEndMs        = millis();
  nightStats.sessionDurationSec  = sessionDurationSec;
  nightStats.wakeDurationSec     = wakeDurationMin * 60UL;
  nightStats.deepSleepDurationSec= deepSleepDurationMin * 60UL;
  nightStats.lightSleepDurationSec = (sessionDurationSec > nightStats.wakeDurationSec + nightStats.deepSleepDurationSec)
                                        ? (sessionDurationSec - nightStats.wakeDurationSec - nightStats.deepSleepDurationSec)
                                        : 0;
  nightStats.lightSleepPercentage= stats.shallowSleepPercentage;
  nightStats.deepSleepPercentage = stats.deepSleepPercentage;
  nightStats.sleepScore          = stats.sleepQualityScore;
  nightStats.avgRespiration      = stats.averageRespiration;
  nightStats.avgHeartRate        = stats.averageHeartbeat;
  nightStats.turnCount           = stats.turnOverCount;
  nightStats.apneaEvents         = stats.apneaEvents;
  nightStats.largeBodyMove       = comp.largeBodyMove;
  nightStats.smallBodyMove       = comp.minorBodyMove;
  nightStats.valid               = true;

  logMsg("[SESSION] Abgeschlossen: " + String(sessionDurationSec / 60) + " Min, Score " + String(nightStats.sleepScore));
  publishNightStats();
}

// ==================== SENSOR-INITIALISIERUNG (nicht-blockierend) ====================
//
// Wichtig: hu.begin() blockiert NICHT mehr endlos in setup(). Schlägt die
// Initialisierung fehl (z.B. Sensor noch nicht bereit, falsche Verkabelung,
// Baudrate), läuft der Rest des Geräts (Webserver, MQTT, WLAN) trotzdem
// normal weiter, und die Initialisierung wird alle 5s im Hintergrund
// wiederholt, bis sie erfolgreich ist.

bool tryInitSensor() {
  Serial.println("Start initialization");
  if (hu.begin() != 0) {
    Serial.println("init error!!!");
    return false;
  }
  Serial.println("Initialization successful");

  Serial.println("Work mode switch");
  if (hu.configWorkMode(hu.eSleepMode) != 0) {
    Serial.println("mode error!!!");
    return false;
  }
  Serial.println("Work mode switch successful");

  switch (hu.getWorkMode()) {
    case 1: Serial.println("Fall detection mode"); break;
    case 2: Serial.println("Sleep detection mode"); break;
    default: Serial.println("Read error");
  }

  hu.configLEDLight(hu.eHPLed, 0); // LED aus im Betrieb
  hu.sensorRet();                  // Reset

  logMsg("[SENSOR] Initialisierung erfolgreich");
  return true;
}

// ==================== SETUP ====================

void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  Serial1.begin(115200, SERIAL_8N1, /*rx=*/7, /*tx=*/6);
#else
  Serial1.begin(115200);
#endif

  // Sensor-Init bewusst SOFORT nach Serial1.begin() und VOR dem WLAN-Aufbau,
  // da startNetwork() bis zu 15s blockieren kann. Der C1001 sendet bereits
  // beim Hochfahren Daten über UART - wartet man zu lange, laufen RX-Puffer
  // voll bzw. veralten, und das Kommando/Antwort-Handshake in hu.begin()
  // kann dann fehlschlagen ("init error!!!"), obwohl die Hardware in Ordnung
  // ist. Vorsichtshalber wird der Puffer vor dem Init-Versuch geleert.
  delay(200); // dem Sensor kurz Zeit geben, nach Power-On bereit zu sein
  while (Serial1.available()) Serial1.read(); // evtl. Boot-/Altdaten verwerfen

  sensorReady = tryInitSensor();
  lastSensorInitAttempt = millis();
  if (!sensorReady) {
    Serial.println("[SENSOR] Initial nicht erreichbar - Init wird im Hintergrund wiederholt");
  }

  startNetwork();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);

  server.on("/",      handleRoot);
  server.on("/sleep",  handleSleep);
  server.on("/json",   handleJSON);
  server.on("/log",    handleLog);
  server.begin();
  Serial.println("[WEB]  Server gestartet (/, /sleep, /json, /log)");

  logMsg("[BOOT] Setup abgeschlossen");
}

// ==================== LOOP ====================

void loop() {
  server.handleClient();
  reconnectWiFi();

  if (!apMode) {
    if (!mqttClient.connected()) reconnectMQTT();
    mqttClient.loop();
  }

  if (!sensorReady) {
    if (millis() - lastSensorInitAttempt > SENSOR_INIT_RETRY_MS) {
      lastSensorInitAttempt = millis();
      sensorReady = tryInitSensor();
    }
    return; // Webserver/MQTT bleiben aktiv, nur Sensor-Auslesung wird übersprungen
  }

  if (millis() - lastSensorRead < SENSOR_READ_INTERVAL_MS) return;
  lastSensorRead = millis();

  // --- Live-Sensordaten holen ---
  sensorData.presence        = hu.smHumanData(hu.eHumanPresence);
  sensorData.movement        = hu.smHumanData(hu.eHumanMovement);
  sensorData.movementParam   = hu.smHumanData(hu.eHumanMovingRange);
  sensorData.respirationRate = hu.getBreatheValue();
  sensorData.breatheState    = hu.getBreatheState();
  sensorData.heartRate       = hu.getHeartRate();
  sensorData.inBed           = hu.smSleepData(hu.eInOrNotInBed);
  sSleepComposite comp       = hu.getSleepComposite();
  sensorData.sleepState      = comp.sleepState;
  sensorData.lastUpdate      = millis();

  // --- Session-State-Machine (Abschnitt 4) ---
  runSessionStateMachine();

  // --- MQTT: jede Sekunde Rohdaten publishen (Abschnitt 7) ---
  publishRawData();

  // --- Serial Monitor Debug-Ausgabe ---
  Serial.print("Presence:     ");
  switch (sensorData.presence) {
    case 0:  Serial.println("Nobody");  break;
    case 1:  Serial.println("Someone"); break;
    default: Serial.println("Error");
  }
  Serial.print("Movement:     ");
  switch (sensorData.movement) {
    case 0:  Serial.println("None"); break;
    case 1:  Serial.println("Still"); break;
    case 2:  Serial.println("Active"); break;
    default: Serial.println("Error");
  }
  Serial.print("Respiration:  "); Serial.println(sensorData.respirationRate);
  Serial.print("Heart rate:   "); Serial.println(sensorData.heartRate);
  Serial.print("In bed:       "); Serial.println(sensorData.inBed);
  Serial.print("Sleep state:  "); Serial.println(sleepStateLabel(sensorData.sleepState));
  Serial.print("Session state:"); Serial.println(sessionState);
  Serial.println("-----------------------");
}

// ==================== WEB-SERVER HANDLER ====================

void handleRoot() {
  unsigned long age = (millis() - sensorData.lastUpdate) / 1000;
  String presenceColor = (sensorData.inBed == 1) ? "#2ecc71" : (sensorData.inBed == 0 ? "#95a5a6" : "#e74c3c");
  String presenceLabel = (sensorData.inBed == 1) ? "In bed"  : (sensorData.inBed == 0 ? "Not in bed" : "Error");

  String html = "<!DOCTYPE html><html lang='de'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='2'><title>Radar - Live</title>";
  html += "<style>"; html += RADAR_CSS; html += "</style>";
  html += "</head><body>";
  html += "<header><svg width='28' height='28' viewBox='0 0 28 28' fill='none'>";
  html += "<circle cx='14' cy='14' r='3' fill='#00c8ff'/>";
  html += "<circle cx='14' cy='14' r='7' stroke='#00c8ff' stroke-width='1.2' stroke-dasharray='3 2' opacity='.6'/>";
  html += "<circle cx='14' cy='14' r='12' stroke='#00c8ff' stroke-width='1' stroke-dasharray='2 3' opacity='.3'/>";
  html += "</svg><h1>Radar Sensor<span>ESP32-C6 / DFRobot C1001</span></h1></header>";
  html += "<div class='nav'><a href='/' class='active'>Live</a><a href='/sleep'>Statistik</a><a href='/json'>JSON</a><a href='/log'>Log</a></div>";

  if (!sensorReady) {
    html += "<div class='grid'><div class='card' style='grid-column:1/-1;border-color:#e74c3c'>";
    html += "<div class='label'>Sensor</div><div style='font-size:1.1rem;color:#e74c3c'>Nicht verbunden - Initialisierung wird im Hintergrund wiederholt (siehe /log)</div>";
    html += "</div></div></body></html>";
    server.send(200, "text/html", html);
    return;
  }

  html += "<div class='grid'>";

  html += "<div class='card'><div class='label'>Anwesenheit</div>";
  html += "<div style='margin:4px 0'><span class='dot' style='background:" + presenceColor + "'></span>";
  html += "<span class='badge' style='background:" + presenceColor + "22;color:" + presenceColor + ";border:1px solid " + presenceColor + "44'>" + presenceLabel + "</span></div></div>";

  String motionColor, motionLabel;
  switch (sensorData.movement) {
    case 0: motionLabel = "None";   motionColor = "#555";    break;
    case 1: motionLabel = "Still";  motionColor = "#f39c12"; break;
    case 2: motionLabel = "Active"; motionColor = "#2ecc71"; break;
    default: motionLabel = "Error"; motionColor = "#e74c3c"; break;
  }
  html += "<div class='card'><div class='label'>Bewegung</div>";
  html += "<div style='margin:4px 0'><span class='badge' style='background:" + motionColor + "22;color:" + motionColor + ";border:1px solid " + motionColor + "44'>" + motionLabel + "</span></div></div>";

  html += "<div class='card'><div class='label'>Herzfrequenz</div>";
  html += "<div style='font-size:2rem'>" + String(sensorData.heartRate) + " bpm</div></div>";

  html += "<div class='card'><div class='label'>Atemfrequenz</div>";
  html += "<div style='font-size:2rem'>" + String(sensorData.respirationRate) + " /min</div></div>";

  html += "<div class='card'><div class='label'>Schlafphase</div>";
  html += "<div style='font-size:1.2rem'>" + String(sleepStateLabel(sensorData.sleepState)) + "</div></div>";

  html += "<div class='card'><div class='label'>Session-Status</div>";
  String sstLabel = (sessionState == OUT_OF_BED) ? "Kein Schlaf aktiv" : (sessionState == IN_BED ? "Session laeuft" : "Kurz aufgestanden (Grace)");
  html += "<div style='font-size:1.1rem'>" + sstLabel + "</div></div>";

  html += "<div class='card'><div class='label'>Letztes Update</div>";
  html += "<div>" + String(age) + " s her</div></div>";

  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleSleep() {
  String html = "<!DOCTYPE html><html lang='de'><head>";
  html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Radar - Schlafstatistik</title><style>"; html += RADAR_CSS; html += "</style></head><body>";
  html += "<header><h1>Schlafstatistik<span>letzte abgeschlossene Session</span></h1></header>";
  html += "<div class='nav'><a href='/'>Live</a><a href='/sleep' class='active'>Statistik</a><a href='/json'>JSON</a><a href='/log'>Log</a></div>";

  if (!nightStats.valid) {
    html += "<div class='grid'><div class='card'>Noch keine abgeschlossene Session vorhanden.</div></div>";
  } else {
    html += "<div class='grid'>";
    html += "<div class='card'><div class='label'>Gesamtdauer</div><div style='font-size:1.4rem'>" + String(nightStats.sessionDurationSec / 60) + " Min</div></div>";
    html += "<div class='card'><div class='label'>Tiefschlaf</div><div style='font-size:1.4rem'>" + String(nightStats.deepSleepDurationSec / 60) + " Min (" + String(nightStats.deepSleepPercentage) + "%)</div></div>";
    html += "<div class='card'><div class='label'>Leichtschlaf</div><div style='font-size:1.4rem'>" + String(nightStats.lightSleepDurationSec / 60) + " Min (" + String(nightStats.lightSleepPercentage) + "%)</div></div>";
    html += "<div class='card'><div class='label'>Wachphasen</div><div style='font-size:1.4rem'>" + String(nightStats.wakeDurationSec / 60) + " Min</div></div>";
    html += "<div class='card'><div class='label'>Schlaf-Score</div><div style='font-size:1.4rem'>" + String(nightStats.sleepScore) + " / 100</div></div>";
    html += "<div class='card'><div class='label'>Ø Herzfrequenz</div><div style='font-size:1.4rem'>" + String(nightStats.avgHeartRate) + " bpm</div></div>";
    html += "<div class='card'><div class='label'>Ø Atemfrequenz</div><div style='font-size:1.4rem'>" + String(nightStats.avgRespiration) + " /min</div></div>";
    html += "<div class='card'><div class='label'>Umdrehungen</div><div style='font-size:1.4rem'>" + String(nightStats.turnCount) + "</div></div>";
    html += "<div class='card'><div class='label'>Apnoe-Ereignisse</div><div style='font-size:1.4rem'>" + String(nightStats.apneaEvents) + "</div></div>";

    // Farbliche Balkenaufteilung (grün=Tief, blau=Leicht, rot=Wach)
    html += "<div class='card' style='grid-column:1/-1'><div class='label'>Verteilung</div>";
    html += "<div class='bar'>";
    html += "<div class='seg-deep' style='width:" + String(nightStats.deepSleepPercentage) + "%'></div>";
    html += "<div class='seg-light' style='width:" + String(nightStats.lightSleepPercentage) + "%'></div>";
    html += "<div class='seg-awake' style='width:" + String(100 - nightStats.deepSleepPercentage - nightStats.lightSleepPercentage) + "%'></div>";
    html += "</div></div>";
    html += "</div>";
  }

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleJSON() {
  String json = "{";
  json += "\"sensorReady\":" + String(sensorReady ? "true" : "false") + ",";
  json += "\"presence\":"     + String(sensorData.presence)        + ",";
  json += "\"movement\":"     + String(sensorData.movement)        + ",";
  json += "\"movementParam\":"+ String(sensorData.movementParam)   + ",";
  json += "\"inBed\":"        + String(sensorData.inBed)           + ",";
  json += "\"sleepState\":"   + String(sensorData.sleepState)      + ",";
  json += "\"respiration\":"  + String(sensorData.respirationRate) + ",";
  json += "\"heartRate\":"    + String(sensorData.heartRate)       + ",";
  json += "\"sessionState\":" + String((int)sessionState)          + ",";
  json += "\"lastUpdateMs\":" + String(sensorData.lastUpdate)      + ",";
  json += "\"nightStatsValid\":" + String(nightStats.valid ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleLog() {
  String out = "";
  if (logFull) {
    for (uint8_t i = 0; i < LOG_BUFFER_SIZE; i++) {
      uint8_t idx = (logIndex + i) % LOG_BUFFER_SIZE;
      out += logBuffer[idx] + "\n";
    }
  } else {
    for (uint8_t i = 0; i < logIndex; i++) {
      out += logBuffer[i] + "\n";
    }
  }
  server.send(200, "text/plain", out);
}