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

unsigned long lastMsg = 0;
unsigned long lastReconnectAttempt = 0;
int dhtPin = 21;
float ultimaDistancia = -999;  // Cambiado a float para comparar distancias
unsigned long ultimaActualizacionLCD = 0;
const long intervaloLCD = 200;  // Aumentado a 500ms para mayor estabilidad

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
  Wire.setClock(100000);  
  
  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Aqui musical    ");
}

void updateLcd(float distancia) {
  unsigned long ahora = millis();
  if (ahora - ultimaActualizacionLCD < intervaloLCD) return;
  ultimaActualizacionLCD = ahora;

  if (abs(distancia - ultimaDistancia) < 30 && ultimaDistancia != -999) return;
  ultimaDistancia = distancia;

  char linea1[17];

  if (distancia < 0) {
    snprintf(linea1, 17, "Buscando...     ");
  } else {
    float cm = distancia / 10.0;
    snprintf(linea1, 17, "Dist: %-6.0f cm ", cm);
  }

  lcd.setCursor(0, 1);
  lcd.print(linea1);
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

int16_t corregirValor(int16_t val) {
  if (val > 10000) return val - 65536;
  return val;
}

int determinarZonaApp(int16_t x, int16_t y) {
  int16_t rx = (int16_t)x;
  int16_t ry = (int16_t)y;

  // ZONA 0
  if (rx >= -900 && rx <= -720 && ry >= -900 && ry <= 0) return 0;

  // ZONA 1
  if (rx >= -3276 && rx <= -576 && ry >= -4536 && ry <= -1062) return 1;

  // ZONA 2
  if (rx >= -3915 && rx <= -774 && ry >= -3573 && ry <= -459) return 2;

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

  // Lógica del radar
  if (ld2450.update()) {
    int numTargets = ld2450.getNrValidTargets();
    float menorDistancia = 99999;
    int indiceMasCercano = -1;

    // Buscar el target más cercano válido
    for (int i = 0; i < numTargets; i++) {
      auto target = ld2450.getTarget(i);
      
      // Descartar valores falsos 0,0 del radar
      if (target.x == 0 && target.y == 0) continue;
      
      float distancia = sqrt(sq((float)target.x) + sq((float)target.y));
      
      if (distancia < menorDistancia) {
        menorDistancia = distancia;
        indiceMasCercano = i;
      }
    }

    // Si no se encontró ningún target real
    if (indiceMasCercano == -1) {
      updateLcd(-1);  // Mostrar "Buscando..."
      return;
    }

    // Obtener el target más cercano
    auto target = ld2450.getTarget(indiceMasCercano);
    float anguloRadianes = atan2((float)target.x, (float)target.y);
    float anguloGrados = anguloRadianes * 180.0 / PI;
    float distanciaTotal = menorDistancia;

    // Crear el JSON (corregida la coma extra)
    String payload = "{";
    payload += "\"id\":" + String(indiceMasCercano) + ",";
    payload += "\"x\":" + String(target.x) + ",";
    payload += "\"y\":" + String(target.y) + ",";
    payload += "\"ang\":" + String(anguloGrados, 1) + ",";
    payload += "\"hum\":" + String(dht.getHumidity(), 1) + ",";
    payload += "\"temp\":" + String(dht.getTemperature(), 1) + ",";
    payload += "\"dist\":" + String(distanciaTotal, 1);  // Sin coma al final
    payload += "}";

    client.publish(miID, payload.c_str());
    Serial.println("Publicado: " + payload);
    
    // Actualizar LCD con la distancia
    updateLcd(distanciaTotal);  
  }

  delay(200);
}
