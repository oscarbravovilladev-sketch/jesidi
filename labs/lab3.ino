#include <Arduino.h>
#include <HardwareSerial.h>
#include <LD2450.hpp>  // Nueva Librería
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

unsigned long lastMsg = 0;
unsigned long lastReconnectAttempt = 0;
int dhtPin = 21;
int ultimosTargets = -1;
unsigned long ultimaActualizacionLCD = 0;
const long intervaloLCD = 500;

const char* miID = "JESIDI-IOT-POC-200574";
const char* mqtt_server = "broker.hivemq.com";

bool i2CAddrTest(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

boolean reconnect() {
  if (client.connect(miID)) {
    Serial.println("¡CONECTADO AL BROKER!");
    return true;
  }
  Serial.print("Falló MQTT, rc=");
  Serial.print(client.state());
  return false;
}

void lcdF() {
  Wire.begin(SDA, SCL);
  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Aqui musical");
}

void updateLcd(int contadorTargets) {
  if (millis() - ultimaActualizacionLCD > intervaloLCD) {
    if (contadorTargets != ultimosTargets) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Radar LD2450");
      lcd.setCursor(0, 1);
      if (contadorTargets <= 0) {
        lcd.print("Buscando...");
      } else {
        lcd.print("Targets: ");
        lcd.print(contadorTargets);
      }
      ultimosTargets = contadorTargets;
    }
    ultimaActualizacionLCD = millis();
  }
}

void initRadar() {
  // Configuración de la nueva librería que te funcionó
  ld2450Serial.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RXP2, TXP2);
  ld2450Serial.setTimeout(1000);
  ld2450.begin(ld2450Serial);

  ld2450.beginConfigurationSession();
  ld2450.setMultiTargetTracking();
  if (ld2450.queryZoneFilter()) {
    // La librería actualiza internamente sus zonas.
    // Como son privadas, las "espiamos" a través de la función de log o las definimos.
    // Nota: Si queryZoneFilter tiene éxito, el radar ya filtrará por hardware
    // si activaste "AreaOnly" en la App.
    Serial.println(">> Zonas sincronizadas con éxito.");
    Serial.println(ld2450.getZoneFilter());
  }
  ld2450.endConfigurationSession();
  Serial.println("--- RADAR (NUEVA LIB) INICIADO ---");
}

int16_t corregirValor(int16_t val) {
  // Si el valor es > 10000, es un número negativo mal interpretado (overflow)
  if (val > 10000) return val - 65536;
  return val;
}

int determinarZonaApp(int16_t x, int16_t y) {
  // 1. Forzamos la interpretación como números con signo de 16 bits
  int16_t rx = (int16_t)x;
  int16_t ry = (int16_t)y;

  // 2. Depuración Crítica: Si esto sale en el monitor, sabrás por qué falla
  // Serial.printf("DEBUG ZONA -> RX: %d | RY: %d\n", rx, ry);

  // ZONA 0 (Ajustada a rangos lógicos)
  if (rx >= -900 && rx <= -720 && ry >= -900 && ry <= 0) return 0;

  // ZONA 1
  if (rx >= -3276 && rx <= -576 && ry >= -4536 && ry <= -1062) return 1;

  // ZONA 2
  if (rx >= -3915 && rx <= -774 && ry >= -3573 && ry <= -459) return 2;

  // Si no cumple NINGUNA de las anteriores, devolverá -1 obligatoriamente
  return -1; 
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(4, INPUT_PULLUP);

  WiFiManager wm;
  if (digitalRead(4) == LOW) {
    Serial.println(">>> MODO CONFIGURACIÓN ACTIVADO");
    wm.resetSettings();
    wm.setConfigPortalTimeout(120);
  }

  if (!wm.autoConnect("ESP32_JESIDI")) {
    Serial.println("Fallo conexión. Reiniciando...");
    delay(3000);
    ESP.restart();
  }

  client.setServer(mqtt_server, 1883);
  client.setBufferSize(512);  // Buffer para evitar crashes
  dht.setup(dhtPin, DHTesp::DHT11);
  initRadar();
  lcdF();
  Serial.println("SETUP COMPLETO");
}

void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) lastReconnectAttempt = 0;
    }
  } else {
    client.loop();
  }

  // Lógica con la nueva librería
  if (ld2450.update()) {
    int numTargets = ld2450.getNrValidTargets();

    // Solo enviamos a MQTT cada 1 segundo para no saturar el broker gratuito
    if (millis() - lastMsg > 1000) {
      for (int i = 0; i < numTargets; i++) {
        auto target = ld2450.getTarget(i);

        // El radar a veces da 0,0 si no hay presencia real
        if (target.x != 0 || target.y != 0) {
          float anguloRadianes = atan2((float)target.x, (float)target.y);
          float anguloGrados = anguloRadianes * 180.0 / PI;
          float distanciaTotal = sqrt(sq((float)target.x) + sq((float)target.y));
          int zona = determinarZonaApp(target.x, target.y);

          String payload = "{";
          payload += "\"id\":" + String(i) + ",";
          payload += "\"x\":" + String(target.x) + ",";
          payload += "\"y\":" + String(target.y) + ",";
          payload += "\"ang\":" + String(anguloGrados, 1) + ",";
          payload += "\"hum\":" + String(dht.getHumidity(), 1) + ",";
          payload += "\"temp\":" + String(dht.getTemperature(), 1) + ",";
          payload += "\"dist\":" + String(distanciaTotal, 1) + ",";
          payload += "\"zona\":" + String(zona);
          payload += "}";

          client.publish(miID, payload.c_str());
          Serial.println("Publicado: " + payload);
        }
      }
      lastMsg = millis();
    }
    updateLcd(numTargets);
  }

  delay(100);  // Pequeña pausa para estabilidad
}
