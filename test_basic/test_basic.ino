
#include "DFRobot_HumanDetection.h"
#include <WiFi.h>
#include <WebServer.h>
#include "arduino_secrets.h"

// ================== WLAN ==================
const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

DFRobot_HumanDetection hu(&Serial1);
WebServer server(80);

// ================== Sensor-Daten (global, vom loop befüllt) ==================
struct SensorData {
  int     presence;      // 0 = niemand, 1 = jemand, -1 = Fehler
  int     fall;          // 0 = keine Bewegung, 1 = still, 2 = aktiv
  int     respirationRate;
  int     heartRate;
  unsigned long lastUpdate;
} sensorData = { -1, -1, 0, 0, 0 };

// ================== Hilfsfunktionen ==================
String presenceText() {
  switch (sensorData.presence) {
    case 0:  return "Niemand anwesend";
    case 1:  return "Person anwesend";
    default: return "Lesefehler";
  }
}

String movementText() {
  switch (sensorData.fall) {
    case 0:  return "Keine Bewegung";
    case 1:  return "Still";
    case 2:  return "Aktiv";
    default: return "Lesefehler";
  }
}

String movementEmoji() {
  switch (sensorData.fall) {
    case 0:  return "&#x26AA;";   // grauer Kreis
    case 1:  return "&#x1F7E1;";  // gelber Kreis
    case 2:  return "&#x1F7E2;";  // gruener Kreis
    default: return "&#x2753;";
  }
}

String presenceColor() { return (sensorData.presence == 1) ? "#2ecc71" : "#95a5a6"; }

// ================== Webserver-Handler ==================
void handleRoot() {
  unsigned long age = (millis() - sensorData.lastUpdate) / 1000;

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='2'>";
  html += "<title>Radar Sensor</title>";
  html += "<style>body{font-family:sans-serif;background:#1a1a2e;color:#eee;text-align:center;padding:20px}";
  html += ".grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:400px;margin:20px auto}";
  html += ".card{background:#16213e;border-radius:12px;padding:16px;border:1px solid #0f3460}";
  html += ".label{font-size:0.7rem;color:#888;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}";
  html += ".value{font-size:1.8rem;font-weight:bold}";
  html += ".badge{display:inline-block;padding:5px 12px;border-radius:8px;font-size:0.9rem;font-weight:bold;color:#fff}";
  html += ".unit{font-size:0.7rem;color:#888;margin-top:4px}";
  html += ".footer{font-size:0.7rem;color:#555;margin-top:16px}</style></head><body>";
  html += "<h2>&#128246; Radar Sensor</h2>";
  html += "<div class='grid'>";

  // Anwesenheit
  html += "<div class='card'><div class='label'>Anwesenheit</div><div class='value'>";
  html += (sensorData.presence == 1) ? "&#x1F7E2;" : "&#x26AA;";
  html += "</div><div class='badge' style='background:";
  html += presenceColor();
  html += "'>";
  html += presenceText();
  html += "</div></div>";

  // Bewegung (Rohwert)
  html += "<div class='card'><div class='label'>Bewegung</div>";
  html += "<div class='value'>" + String(sensorData.fall) + "</div>";
  html += "<div class='unit'>Bewegungsparameter</div></div>";

  // Atemfrequenz
  html += "<div class='card'><div class='label'>Atemfrequenz</div>";
  html += "<div class='value'>" + String(sensorData.respirationRate) + "</div>";
  html += "<div class='unit'>Atemz&uuml;ge / min</div></div>";

  // Herzfrequenz
  html += "<div class='card'><div class='label'>Herzfrequenz</div>";
  html += "<div class='value'>" + String(sensorData.heartRate) + "</div>";
  html += "<div class='unit'>bpm</div></div>";

  html += "</div>";
  html += "<p class='footer'>Letzte Messung vor " + String(age) + " s &nbsp;|&nbsp; ESP32 / DFRobot Radar</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// JSON-Endpunkt für einfache Weiterverarbeitung
void handleJSON() {
  String json = "{";
  json += "\"presence\":"       + String(sensorData.presence)       + ",";
  json += "\"fall\":"           + String(sensorData.fall)           + ",";
  json += "\"respirationRate\":" + String(sensorData.respirationRate) + ",";
  json += "\"heartRate\":"      + String(sensorData.heartRate)      + ",";
  json += "\"ageMs\":"          + String(millis() - sensorData.lastUpdate);
  json += "}";
  server.send(200, "application/json", json);
}

// ================== WLAN ==================
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("[WIFI] Verbinde...");
  WiFi.begin(ssid, password);
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WIFI] Verbunden, IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WEB]  http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WIFI] NICHT verbunden");
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

#if defined(ESP32)
  Serial1.begin(115200, SERIAL_8N1, /*rx=*/7, /*tx=*/6);
#else
  Serial1.begin(115200);
#endif

  connectWiFi();

  // Webserver-Routen registrieren
  server.on("/",     handleRoot);
  server.on("/json", handleJSON);
  server.begin();
  Serial.println("[WEB]  Server gestartet");

  // Radar initialisieren
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
  // Webserver-Anfragen bedienen (non-blocking)
  server.handleClient();

  // WiFi ggf. wiederverbinden
  connectWiFi();

  // Sensordaten lesen und global speichern
  sensorData.presence       = hu.smHumanData(hu.eHumanPresence);
  sensorData.fall           = hu.smHumanData(hu.eHumanMovement);  // 1 = Active = möglicher Sturz im SleepMode
  sensorData.respirationRate = hu.getBreatheValue();
  sensorData.heartRate      = hu.getHeartRate();
  sensorData.lastUpdate     = millis();

  // Serielle Ausgabe (wie bisher)
  Serial.print("Anwesenheit: ");
  switch (sensorData.presence) {
    case 0:  Serial.println("Niemand"); break;
    case 1:  Serial.println("Jemand");  break;
    default: Serial.println("Fehler");
  }
  Serial.print("Sturz/Bewegung: ");
  Serial.println(sensorData.fall);
  Serial.print("Atemfrequenz: ");
  Serial.println(sensorData.respirationRate);
  Serial.print("Herzfrequenz: ");
  Serial.println(sensorData.heartRate);
  Serial.println("-----------------------");

  delay(1000);
}