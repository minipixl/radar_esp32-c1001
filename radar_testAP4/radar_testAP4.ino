/*********************************************************************
 * Radar-Schlafüberwachung mit ESP32-C6 SuperMini + DFRobot C1001 + MQTT
 * Umsetzung der Funktionalen Spezifikation funct_spec2.md
 *
 * Benötigte Libraries (über Bibliotheksverwalter bzw. GitHub):
 *   - DFRobot_HumanDetection   (DFRobot, github.com/DFRobot/DFRobot_HumanDetection)
 *   - PubSubClient             (Nick O'Leary, Bibliotheksverwalter)
 *   - WiFi                     (im ESP32-Board-Package enthalten)
 *   - WebServer                (im ESP32-Board-Package enthalten)
 *   - time.h                   (Teil des ESP32-Cores, keine Installation nötig)
 *
 * Alle sSleepStatistics/sSleepComposite-Feldnamen und smSleepData()-Enums
 * wurden am 2026-08 gegen die reale DFRobot_HumanDetection.h/.cpp von
 * github.com/DFRobot/DFRobot_HumanDetection (Branch "master") geprüft.
 *********************************************************************/

#include "DFRobot_HumanDetection.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <WebServer.h>
#include <time.h>              // NTP / Echtzeit (Abschnitt 2)
#include "esp_system.h"        // esp_reset_reason() - Diagnose fuer naechtliche Reboots
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
const uint32_t INTERRUPTION_WINDOW_MS   = 10UL * 60UL * 1000UL;  // 10 Min. Toleranz für kurzes Aufstehen
const uint32_t MIN_DURATION_FOR_SESSION = 20UL * 60UL * 1000UL; // Mindestlänge, damit eine Session zählt (20 Min.)
const uint32_t SENSOR_READ_INTERVAL_MS  = 1000UL;               // 1x pro Sekunde messen/publishen

// --- Echtzeit / NTP (Abschnitt 2) ---
const char* NTP_SERVER_1 = "de.pool.ntp.org";
const char* NTP_SERVER_2 = "pool.ntp.org";
// POSIX-TZ-String für Deutschland/Schweiz (MEZ/MESZ inkl. automatischer Sommerzeit-Umstellung)
const char* TZ_INFO = "CET-1CEST,M3.5.0,M10.5.0/3";

// --- Ringpuffer für /log ---
const uint16_t LOG_BUFFER_SIZE = 500;

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
  ".seg-deep{background:#2ecc71}.seg-light{background:#3498db}.seg-awake{background:#e74c3c}"
  ".btnrow{display:flex;gap:10px;padding:0 20px 16px}"
  ".btn{flex:1;text-align:center;padding:12px;border-radius:8px;text-decoration:none;font-weight:600;font-size:.9rem}"
  ".btn-start{background:#2ecc7122;color:#2ecc71;border:1px solid #2ecc7166}"
  ".btn-stop{background:#e74c3c22;color:#e74c3c;border:1px solid #e74c3c66}"
  ".manual-badge{display:inline-block;margin-left:8px;padding:2px 8px;border-radius:20px;font-size:.7rem;background:#f39c1222;color:#f39c12;border:1px solid #f39c1244}";

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
  String   sessionStartTimeStr   = "--:--:--"; // Abschnitt 5: Uhrzeit statt Sekunden
  String   sessionEndTimeStr     = "--:--:--";
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
  bool     manualSession         = false; // true, falls per Knopf gestartet/beendet
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
String currentSessionStartTimeStr = "--:--:--";

// Manuelle Übersteuerung (Abschnitt 4): true, solange die automatische
// inBed-gesteuerte State-Machine pausiert ist und Start/Stop nur noch über
// die Webserver-Buttons /start bzw. /stop erfolgen.
bool manualControlActive = false;
bool timeSynced = false;

String logBuffer[LOG_BUFFER_SIZE];
uint8_t logIndex = 0;
bool logFull = false;

bool apMode = false;
bool sensorReady = false;
unsigned long lastWifiRetry = 0;
unsigned long lastSensorRead = 0;
unsigned long lastSensorInitAttempt = 0;
const uint32_t SENSOR_INIT_RETRY_MS = 5000; // alle 5s erneut versuchen, falls Sensor nicht antwortet

// --- Diagnose fuer naechtliche Reboots ---
unsigned long lastHeapLog = 0;
const uint32_t HEAP_LOG_INTERVAL_MS = 5UL * 60UL * 1000UL; // alle 5 Min. Heap-Status loggen

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

// Aktuelle Uhrzeit als HH:MM:SS (24h), Abschnitt 2. Liefert "--:--:--",
// solange noch kein NTP-Sync stattgefunden hat (z.B. im AP-Fallback ohne
// Internetzugang).
String nowTimeStr() {
  if (!timeSynced) return "--:--:--";
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 50)) return "--:--:--";
  char buf[9];
  strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
  return String(buf);
}

// Sekunden -> HH:MM:SS Dauer-Format (Abschnitt 2/5: "hh:mm:ss statt Sekunden").
String secToHms(uint32_t totalSeconds) {
  uint32_t h = totalSeconds / 3600UL;
  uint32_t m = (totalSeconds % 3600UL) / 60UL;
  uint32_t s = totalSeconds % 60UL;
  char buf[10];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
  return String(buf);
}

void logMsg(const String& msg) {
  String line = "[" + nowTimeStr() + "] " + msg;
  Serial.println(line);
  logBuffer[logIndex] = line;
  logIndex = (logIndex + 1) % LOG_BUFFER_SIZE;
  if (logIndex == 0) logFull = true;
  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_LOG_TOPIC, line.c_str());
  }
}

// Loggt den Grund des letzten Resets/Neustarts. Wird ganz am Anfang von
// setup() aufgerufen, damit der Eintrag im /log-Ringpuffer erhalten bleibt,
// auch wenn WLAN/MQTT/Sensor-Init danach fehlschlagen. So laesst sich am
// naechsten Morgen nachvollziehen, WARUM das Geraet neu gestartet ist:
//   - ESP_RST_BROWNOUT  -> Stromversorgungsproblem (Netzteil/Kabel/Spitzen)
//   - ESP_RST_TASK_WDT / ESP_RST_INT_WDT -> irgendwo blockierender Code
//     (z.B. haengender UART-Handshake mit dem C1001)
//   - ESP_RST_PANIC     -> Exception, z.B. durch Heap-Corruption/Fragmentierung
//   - ESP_RST_POWERON   -> echter Stromausfall/Power-Cycle (nicht Software)
void logResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  const char* reasonStr;
  switch (reason) {
    case ESP_RST_POWERON:   reasonStr = "Power-On (Stromausfall/-cycle)"; break;
    case ESP_RST_EXT:       reasonStr = "Externer Reset (Pin/Taster)"; break;
    case ESP_RST_SW:        reasonStr = "Software-Reset (z.B. ESP.restart())"; break;
    case ESP_RST_PANIC:     reasonStr = "Exception/Panic (z.B. Heap-Corruption)"; break;
    case ESP_RST_INT_WDT:   reasonStr = "Interrupt-Watchdog (blockierender Code)"; break;
    case ESP_RST_TASK_WDT:  reasonStr = "Task-Watchdog (Task haengt zu lange)"; break;
    case ESP_RST_WDT:       reasonStr = "Sonstiger Watchdog-Reset"; break;
    case ESP_RST_DEEPSLEEP: reasonStr = "Aufwachen aus Deep Sleep"; break;
    case ESP_RST_BROWNOUT:  reasonStr = "Brownout (Spannungseinbruch!)"; break;
    case ESP_RST_SDIO:      reasonStr = "SDIO-Reset"; break;
    default:                reasonStr = "Unbekannt"; break;
  }
  logMsg("[BOOT] Reset-Grund: " + String(reasonStr) + " (Code " + String((int)reason) + ")");
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

    // NTP-Zeitsync (Abschnitt 2) - nur möglich mit Internetzugang, also nicht im AP-Fallback.
    Serial.println("[NTP]  Synchronisiere Uhrzeit...");
    configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10000)) { // bis zu 10s auf Sync warten
      timeSynced = true;
      char buf[9];
      strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
      Serial.print("[NTP]  Uhrzeit synchronisiert: ");
      Serial.println(buf);
    } else {
      Serial.println("[NTP]  Sync fehlgeschlagen, versuche es spaeter erneut");
    }
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
    if (!timeSynced) {
      configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 5000)) timeSynced = true;
    }
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

  // Bewusst mit char-Puffer + snprintf statt String-Konkatenation: diese
  // Funktion laeuft die ganze Nacht 1x/Sekunde. String += erzeugt dabei
  // jedes Mal mehrere temporaere Heap-Allokationen, was über 8h zu
  // Heap-Fragmentierung und im schlimmsten Fall zum Absturz/Reboot fuehren
  // kann. snprintf schreibt direkt in einen Stack-Puffer, kein Heap-Churn.
  char payload[220];
  snprintf(payload, sizeof(payload),
    "{\"presence\":%d,\"movement\":%d,\"inBed\":%d,\"sleepState\":%d,"
    "\"breatheState\":%d,\"respiration\":%.1f,\"heartRate\":%.1f,\"millis\":%lu}",
    sensorData.presence, sensorData.movement, sensorData.inBed, sensorData.sleepState,
    sensorData.breatheState, sensorData.respirationRate, sensorData.heartRate,
    (unsigned long)millis());

  mqttClient.publish(MQTT_TOPIC, payload);
}

void publishNightStats() {
  if (!mqttClient.connected() || !nightStats.valid) return;

  String payload = "{";
  payload += "\"sessionStartTime\":\""   + nightStats.sessionStartTimeStr    + "\",";
  payload += "\"sessionEndTime\":\""     + nightStats.sessionEndTimeStr      + "\",";
  payload += "\"manualSession\":"        + String(nightStats.manualSession ? "true" : "false") + ",";
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
// Zustandsübergänge (automatischer Modus):
//   OUT_OF_BED  --inBed==1-->  IN_BED           (neue Session startet)
//   IN_BED      --inBed==0-->  GRACE_EXIT        (Session pausiert, Toleranzfenster startet)
//   GRACE_EXIT  --inBed==1-->  IN_BED            (Session läuft weiter, kein Abbruch)
//   GRACE_EXIT  --Timeout(5min)-->  OUT_OF_BED   (Session wird abgeschlossen & ausgewertet)
//
// Dadurch werden kurze Aufstehphasen (<5 Min, z.B. Toilette) nicht mehr als
// Sessionende fehlinterpretiert.
//
// MANUELLE ÜBERSTEUERUNG: Solange manualControlActive true ist (per /start
// bzw. /stop im Live-Dashboard gesetzt), wird diese automatische Logik
// komplett übersprungen. Start und Ende der Session werden dann
// ausschließlich durch die Buttons bestimmt - der Sensor liefert in dieser
// Zeit weiterhin Live-Werte (Herzfrequenz, Atmung etc.), aber inBed-Wechsel
// lösen keine Zustandsübergänge mehr aus.

void runSessionStateMachine() {
  if (manualControlActive) return; // Automatik pausiert während manueller Steuerung

  bool inBedNow = (sensorData.inBed == 1);

  switch (sessionState) {

    case OUT_OF_BED:
      if (inBedNow) {
        sessionStartMs = millis();
        currentSessionStartTimeStr = nowTimeStr();
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

  // Bei manuell gestoppten Sessions greift die Mindestdauer nicht - wenn der
  // Nutzer bewusst "Stop" drückt, soll das respektiert werden, auch wenn die
  // Session kürzer als MIN_DURATION_FOR_SESSION war.
  if (!manualControlActive && sessionDurationSec < (MIN_DURATION_FOR_SESSION / 1000)) {
    logMsg("[SESSION] Zu kurz (" + secToHms(sessionDurationSec) + "), wird verworfen");
    return;
  }

  // Laut Spec: eWakeDuration / eDeepSleepDuration (smSleepData) statt
  // sSleepStatistics.sleepTime (kumulativer Zähler) nutzen. Der Sensor
  // liefert diese beiden Werte in MINUTEN (siehe DFRobot_HumanDetection.cpp,
  // case eWakeDuration / eDeepSleepDuration).
  //
  // WICHTIG (manuelle Übersteuerung): Diese Werte berechnet der Sensor
  // intern anhand SEINES EIGENEN Erkennungszeitraums - sie lassen sich durch
  // die Start/Stop-Buttons NICHT zurücksetzen oder exakt an unsere manuell
  // definierten Grenzen anpassen. Bei manuellem Start/Stop können Wake-/
  // Deep-Sleep-Aufteilung und Score also von der tatsächlich manuell
  // gewählten Zeitspanne abweichen - nur sessionStart/-EndZeit und die
  // Gesamtdauer sind bei manueller Steuerung exakt.
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
  nightStats.sessionStartTimeStr = currentSessionStartTimeStr;
  nightStats.sessionEndTimeStr   = nowTimeStr();
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
  nightStats.manualSession       = manualControlActive;
  nightStats.valid               = true;

  logMsg("[SESSION] Abgeschlossen (" + nightStats.sessionStartTimeStr + " - " + nightStats.sessionEndTimeStr +
         ", " + secToHms(sessionDurationSec) + "), Score " + String(nightStats.sleepScore));
  publishNightStats();
}

// ==================== MANUELLE ÜBERSTEUERUNG (Abschnitt 4) ====================
//
// /start: Beginnt eine Session manuell (falls keine läuft) bzw. übernimmt
//         eine bereits automatisch laufende Session in die manuelle
//         Steuerung, ohne deren Startzeit zu verwerfen.
// /stop:  Beendet die aktuell laufende Session sofort (unabhängig davon, ob
//         sie automatisch oder manuell gestartet wurde) und wertet sie aus.
//         Danach kehrt das Gerät wieder in den automatischen Modus zurück.

void handleStartSession() {
  if (sessionState == OUT_OF_BED) {
    sessionStartMs = millis();
    currentSessionStartTimeStr = nowTimeStr();
    sessionState = IN_BED;
    logMsg("[SESSION] Manueller Start (" + currentSessionStartTimeStr + ")");
  } else {
    logMsg("[SESSION] Manuelle Übersteuerung uebernimmt laufende Session");
  }
  manualControlActive = true;

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStopSession() {
  if (sessionState != OUT_OF_BED) {
    finalizeSession();
  } else {
    logMsg("[SESSION] Manueller Stop ohne laufende Session ignoriert");
  }
  sessionState = OUT_OF_BED;
  manualControlActive = false;

  server.sendHeader("Location", "/");
  server.send(303);
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

  // Ganz am Anfang loggen, VOR allen anderen Init-Schritten, damit der
  // Reset-Grund auch dann im Ringpuffer landet, wenn Sensor-Init oder WLAN
  // danach haengen/fehlschlagen sollten.
  logResetReason();

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
  server.on("/start",  handleStartSession);
  server.on("/stop",   handleStopSession);
  server.begin();
  Serial.println("[WEB]  Server gestartet (/, /sleep, /json, /log, /start, /stop)");

  logMsg("[BOOT] Setup abgeschlossen");
}

// ==================== LOOP ====================

void loop() {
  server.handleClient();
  reconnectWiFi();

  // Heap-Trend alle 5 Min. loggen. Faellt der Wert ueber die Nacht stetig,
  // ist Heap-Fragmentierung durch String-Konkatenation die Ursache fuer die
  // Reboots - getMinFreeHeap() zeigt den bisherigen Tiefpunkt seit Boot.
  if (millis() - lastHeapLog > HEAP_LOG_INTERVAL_MS) {
    lastHeapLog = millis();
    logMsg("[HEAP] frei=" + String(ESP.getFreeHeap()) + " min=" + String(ESP.getMinFreeHeap()));
  }

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

  html += "<div class='btnrow'>";
  html += "<a href='/start' class='btn btn-start'>&#9654; Start Session</a>";
  html += "<a href='/stop' class='btn btn-stop'>&#9632; Stop Session</a>";
  html += "</div>";

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
  html += "<div style='font-size:1.1rem'>" + sstLabel;
  if (manualControlActive) html += "<span class='manual-badge'>MANUELL</span>";
  html += "</div></div>";

  if (sessionState != OUT_OF_BED) {
    html += "<div class='card'><div class='label'>Session gestartet</div>";
    html += "<div style='font-size:1.2rem'>" + currentSessionStartTimeStr + " Uhr</div></div>";
  }

  html += "<div class='card'><div class='label'>Aktuelle Uhrzeit</div>";
  html += "<div style='font-size:1.2rem'>" + nowTimeStr() + " Uhr</div></div>";

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
    html += "<div class='card'><div class='label'>Beginn</div><div style='font-size:1.4rem'>" + nightStats.sessionStartTimeStr + " Uhr</div></div>";
    html += "<div class='card'><div class='label'>Ende</div><div style='font-size:1.4rem'>" + nightStats.sessionEndTimeStr + " Uhr</div></div>";
    if (nightStats.manualSession) {
      html += "<div class='card'><div class='label'>Erfassung</div><div style='font-size:1.1rem'><span class='manual-badge'>MANUELL</span></div></div>";
    }
    html += "<div class='card'><div class='label'>Gesamtdauer</div><div style='font-size:1.4rem'>" + secToHms(nightStats.sessionDurationSec) + "</div></div>";
    html += "<div class='card'><div class='label'>Tiefschlaf</div><div style='font-size:1.4rem'>" + secToHms(nightStats.deepSleepDurationSec) + " (" + String(nightStats.deepSleepPercentage) + "%)</div></div>";
    html += "<div class='card'><div class='label'>Leichtschlaf</div><div style='font-size:1.4rem'>" + secToHms(nightStats.lightSleepDurationSec) + " (" + String(nightStats.lightSleepPercentage) + "%)</div></div>";
    html += "<div class='card'><div class='label'>Wachphasen</div><div style='font-size:1.4rem'>" + secToHms(nightStats.wakeDurationSec) + "</div></div>";
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
  json += "\"currentTime\":\"" + nowTimeStr() + "\",";
  json += "\"timeSynced\":" + String(timeSynced ? "true" : "false") + ",";
  json += "\"manualControlActive\":" + String(manualControlActive ? "true" : "false") + ",";
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
    for (uint16_t i = 0; i < LOG_BUFFER_SIZE; i++) {
      uint16_t idx = (logIndex + i) % LOG_BUFFER_SIZE;
      out += logBuffer[idx] + "\n";
    }
  } else {
    for (uint16_t i = 0; i < logIndex; i++) {
      out += logBuffer[i] + "\n";
    }
  }
  server.send(200, "text/plain", out);
}