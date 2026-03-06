#include <Arduino.h>
#include <HardwareSerial.h>
#include <LD2450.hpp>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "DHTesp.h"

using namespace esphome::ld245x;

#define RXP2 16
#define TXP2 17
#define SDA 14
#define SCL 13

HardwareSerial ld2450Serial(2);
LD2450 ld2450;
LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiClient espClient;
PubSubClient client(espClient);
DHTesp dht;

unsigned long lastReconnectAttempt = 0;
unsigned long lastDhtRead = 0;
const long intervaloDHT = 2000;
int dhtPin = 21;
float distanciaSuavizada = -1;
const float EMA_ALPHA = 0.15;
float cachedTemp = 0;
float cachedHum = 0;
char mqttBuffer[160];

const char* miID = "JESIDI-IOT-POC-200574";
const char* mqtt_server = "broker.hivemq.com";

bool i2CAddrTest(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

boolean reconnect() {
  if (client.connect(miID)) {
    Serial.println("¡CONECTADO AL BROKER!");
    updateLcdStatus("MQTT: OK        ");
    return true;
  }
  Serial.print("Falló MQTT, rc=");
  Serial.print(client.state());
  updateLcdStatus("MQTT: Error     ");
  return false;
}

void lcdF() {
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);  
  
  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Aqui musical    ");
}

void updateLcdStatus(const char* msg) {
  lcd.setCursor(0, 1);
  lcd.print(msg);
}

void initRadar() {
  ld2450Serial.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RXP2, TXP2);
  ld2450Serial.setTimeout(1000);
  ld2450.begin(ld2450Serial);

  ld2450.beginConfigurationSession();
  ld2450.setMultiTargetTracking();
  if (ld2450.queryZoneFilter()) {
    Serial.println(">> Zonas sincronizadas con éxito.");
    Serial.println(ld2450.getZoneFilter());
  }
  ld2450.endConfigurationSession();
  Serial.println("--- RADAR (NUEVA LIB) INICIADO ---");
}

void updateDht() {
  unsigned long ahora = millis();
  if (ahora - lastDhtRead < intervaloDHT) return;
  lastDhtRead = ahora;
  float t = dht.getTemperature();
  float h = dht.getHumidity();
  if (!isnan(t)) cachedTemp = t;
  if (!isnan(h)) cachedHum = h;
}

unsigned long btnPressStart = 0;
const long BTN_LONG_PRESS = 3000;  // 3 segundos para reset WiFi

void checkWiFiResetButton() {
  if (digitalRead(4) == LOW) {
    if (btnPressStart == 0) btnPressStart = millis();
    long elapsed = millis() - btnPressStart;
    int secsLeft = (BTN_LONG_PRESS - elapsed) / 1000 + 1;
    char buf[17];
    snprintf(buf, 17, "Reset WiFi: %ds  ", secsLeft);
    updateLcdStatus(buf);
    if (elapsed >= BTN_LONG_PRESS) {
      Serial.println(">>> RESET WIFI POR BOTÓN");
      updateLcdStatus("WiFi: Reset...  ");
      WiFiManager wm;
      wm.resetSettings();
      delay(1000);
      ESP.restart();
    }
  } else {
    if (btnPressStart > 0) {
      // Soltó antes de tiempo, restaurar estado
      updateLcdStatus(client.connected() ? "MQTT: OK        " : "MQTT: Error     ");
    }
    btnPressStart = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(4, INPUT_PULLUP);

  WiFiManager wm;
  wm.setConfigPortalTimeout(120);
  if (!wm.autoConnect("ESP32_JESIDI")) {
    Serial.println("Fallo conexión. Reiniciando...");
    updateLcdStatus("WiFi: Error     ");
    delay(3000);
    ESP.restart();
  }

  updateLcdStatus("WiFi: OK        ");
  client.setServer(mqtt_server, 1883);
  client.setBufferSize(512);
  dht.setup(dhtPin, DHTesp::DHT11);
  initRadar();
  lcdF();
  
  // Mostrar mensaje inicial estable
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("Radar LD2450    ");
  lcd.setCursor(0, 1);
  lcd.print("Iniciando...    ");
  
  Serial.println("SETUP COMPLETO");
}

void loop() {
  // Reconexión MQTT
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) lastReconnectAttempt = 0;
    }
  } else {
    client.loop();
  }

  updateDht();
  checkWiFiResetButton();

  // Lógica del radar
  if (ld2450.update()) {
    int numTargets = ld2450.getNrValidTargets();
    float menorDistSq = 1e18;
    int indiceMasCercano = -1;

    for (int i = 0; i < numTargets; i++) {
      auto target = ld2450.getTarget(i);
      if (target.x == 0 && target.y == 0) continue;
      float dSq = sq((float)target.x) + sq((float)target.y);
      if (dSq < menorDistSq) {
        menorDistSq = dSq;
        indiceMasCercano = i;
      }
    }

    if (indiceMasCercano == -1) return;

    auto target = ld2450.getTarget(indiceMasCercano);
    float distanciaReal = sqrt(menorDistSq);
    float anguloGrados = atan2((float)target.x, (float)target.y) * 180.0 / PI;

    // Suavizado EMA
    if (distanciaSuavizada < 0) {
      distanciaSuavizada = distanciaReal;
    } else {
      distanciaSuavizada = EMA_ALPHA * distanciaReal + (1 - EMA_ALPHA) * distanciaSuavizada;
    }

    snprintf(mqttBuffer, sizeof(mqttBuffer),
      "{\"id\":%d,\"x\":%d,\"y\":%d,\"ang\":%.1f,\"hum\":%.1f,\"temp\":%.1f,\"dist\":%.1f}",
      indiceMasCercano, target.x, target.y, anguloGrados,
      cachedHum, cachedTemp, distanciaSuavizada);

    client.publish(miID, mqttBuffer);
    Serial.println(mqttBuffer);
  }
}
