#include <Arduino.h>
#include <HardwareSerial.h>
#include <LD2450.hpp>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include "Freenove_WS2812_Lib_for_ESP32.h"

using namespace esphome::ld245x;

#define RXP2 16
#define TXP2 17
HardwareSerial ld2450Serial(2);
LD2450 ld2450;
WiFiClient espClient;
PubSubClient client(espClient);
DHTesp dht;

#define LED_PIN    42
#define LED_COUNT  8
#define LED_CHANNEL 0
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LED_COUNT, LED_PIN, LED_CHANNEL, TYPE_GRB);

// --- Configuracion ---
const int dhtPin = 21;
const long intervaloDHT = 2000;
const float DIST_UMBRAL = 15.0;   // mm de cambio minimo para publicar
const float ANG_UMBRAL = 1.5;     // grados de cambio minimo para publicar
const long MQTT_MIN_INTERVAL = 50; // ms minimo entre publishes (max ~20/seg)
const long MQTT_HEARTBEAT = 500;   // ms forzar publish aunque no cambie (heartbeat)

// --- Estado ---
unsigned long lastReconnectAttempt = 0;
unsigned long lastDhtRead = 0;
unsigned long lastValidTarget = 0;
unsigned long lastMqttPublish = 0;
float lastPublishedDist = -1;
float lastPublishedAng = -999;
int lastLedCount = -1;
float cachedTemp = 0;
float cachedHum = 0;
char mqttBuffer[160];

const char* miID = "JESIDI-IOT-POC-200574";
const char* mqtt_server = "broker.hivemq.com";

boolean reconnect() {
  if (client.connect(miID)) {
    Serial.println("MQTT: OK");
    return true;
  }
  Serial.print("MQTT fail rc=");
  Serial.println(client.state());
  return false;
}

// --- WS2812 ---
void ledsEstado(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) strip.setLedColorData(i, r, g, b);
  strip.show();
}

void ledsDistancia(float distMm) {
  int enc = 0;
  if (distMm > 0) {
    enc = (int)(distMm / 625) + 1;
    if (enc > LED_COUNT) enc = LED_COUNT;
  }
  // Solo actualizar strip si cambio el numero de LEDs
  if (enc == lastLedCount) return;
  lastLedCount = enc;
  for (int i = 0; i < LED_COUNT; i++) strip.setLedColorData(i, 0, 0, 0);
  for (int i = 0; i < enc; i++) strip.setLedColorData(i, 0, 255, 0);
  strip.show();
}

// --- Radar ---
void initRadar() {
  ld2450Serial.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RXP2, TXP2);
  ld2450Serial.setTimeout(1000);
  ld2450.begin(ld2450Serial);
  ld2450.beginConfigurationSession();
  ld2450.setSingleTargetTracking();
  if (ld2450.queryZoneFilter()) {
    Serial.println("Zonas OK");
    Serial.println(ld2450.getZoneFilter());
  }
  ld2450.endConfigurationSession();
  Serial.println("Radar OK");
}

// --- DHT ---
void updateDht() {
  unsigned long ahora = millis();
  if (ahora - lastDhtRead < intervaloDHT) return;
  lastDhtRead = ahora;
  float t = dht.getTemperature();
  float h = dht.getHumidity();
  if (!isnan(t)) cachedTemp = t;
  if (!isnan(h)) cachedHum = h;
}

// --- Boton reset WiFi ---
unsigned long btnPressStart = 0;
const long BTN_LONG_PRESS = 3000;

void blinkAzul() {
  static unsigned long lastBlink = 0;
  static bool on = false;
  unsigned long ahora = millis();
  if (ahora - lastBlink >= 500) {
    lastBlink = ahora;
    on = !on;
    ledsEstado(0, 0, on ? 255 : 0);
  }
}

void checkWiFiResetButton() {
  if (digitalRead(4) == LOW) {
    if (btnPressStart == 0) btnPressStart = millis();
    if (millis() - btnPressStart >= BTN_LONG_PRESS) {
      Serial.println("RESET WIFI -> Portal de configuracion");
      ledsEstado(0, 0, 255);  // azul fijo = entrando en modo config

      WiFiManager wm;
      wm.resetSettings();
      wm.setConfigPortalTimeout(120);
      wm.setAPCallback([](WiFiManager* wm) {
        Serial.println("Portal WiFi abierto - LED azul parpadeante");
      });

      // Parpadeo azul durante el portal (bloqueante con ticker manual)
      bool configured = false;
      // Lanzar portal en modo no-bloqueante no es viable, usamos startConfigPortal bloqueante
      // pero WiFiManager permite setWebServerCallback para parpadear
      // La forma mas simple: usar startConfigPortal que es bloqueante
      configured = wm.startConfigPortal("ESP32_JESIDI");

      if (configured) {
        Serial.println("WiFi reconfigurado OK");
        ledsEstado(0, 255, 180);  // cian = exito
        delay(1000);
        ESP.restart();  // reiniciar para reconectar limpiamente
      } else {
        Serial.println("Config portal timeout, reiniciando...");
        ledsEstado(255, 0, 0);  // rojo = fallo
        delay(1000);
        ESP.restart();
      }
    }
  } else {
    btnPressStart = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(4, INPUT_PULLUP);

  strip.begin();
  strip.setBrightness(20);
  ledsEstado(255, 0, 0);       // rojo = iniciando

  ledsEstado(255, 100, 0);     // naranja = conectando WiFi
  WiFiManager wm;
  wm.setConfigPortalTimeout(120);
  wm.setAPCallback([](WiFiManager* wm) {
    Serial.println("Portal WiFi abierto - esperando configuracion (azul)");
    ledsEstado(0, 0, 255);     // azul = esperando config WiFi
  });
  if (!wm.autoConnect("ESP32_JESIDI")) {
    Serial.println("WiFi fail, reiniciando...");
    ledsEstado(255, 0, 0);     // rojo = fallo
    delay(3000);
    ESP.restart();
  }

  ledsEstado(0, 255, 180);     // cian = WiFi OK
  client.setServer(mqtt_server, 1883);
  client.setBufferSize(512);
  dht.setup(dhtPin, DHTesp::DHT11);
  initRadar();

  ledsEstado(0, 255, 180);     // cian = listo
  delay(500);
  ledsEstado(0, 0, 0);
  Serial.println("SETUP OK");
}

void loop() {
  // MQTT reconexion no bloqueante
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) lastReconnectAttempt = 0;
    }
  } else {
    client.loop();
  }

  updateDht();
  checkWiFiResetButton();

  if (!ld2450.update()) return;

  // Single target mode: solo hay 1 target
  auto target = ld2450.getTarget(0);
  if (target.x == 0 && target.y == 0) {
    // Sin target: mantener ultimo valor de LEDs (no apagar)
    return;
  }

  lastValidTarget = millis();
  float dist = sqrt(sq((float)target.x) + sq((float)target.y));
  float ang = atan2((float)target.x, (float)target.y) * 180.0 / PI;

  // MQTT: publicar solo si cambio significativo o heartbeat
  unsigned long ahora = millis();
  if (ahora - lastMqttPublish < MQTT_MIN_INTERVAL) return;

  bool cambio = abs(dist - lastPublishedDist) > DIST_UMBRAL ||
                abs(ang - lastPublishedAng) > ANG_UMBRAL;
  bool heartbeat = ahora - lastMqttPublish >= MQTT_HEARTBEAT;

  if (!cambio && !heartbeat) return;

  snprintf(mqttBuffer, sizeof(mqttBuffer),
    "{\"id\":%d,\"x\":%d,\"y\":%d,\"ang\":%.1f,\"hum\":%.1f,\"temp\":%.1f,\"dist\":%.0f}",
    0, target.x, target.y, ang, cachedHum, cachedTemp, dist);

  client.publish(miID, mqttBuffer);
  ledsDistancia(dist);  // LEDs = exactamente lo que se publica
  lastMqttPublish = ahora;
  lastPublishedDist = dist;
  lastPublishedAng = ang;
}
