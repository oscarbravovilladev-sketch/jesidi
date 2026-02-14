#include <WiFi.h>
#include <WiFiManager.h> 
#include <PubSubClient.h>
#include <UltrasonicSensor.h>
#include "DHTesp.h"

// --- CONFIGURACIÓN ÚNICA ---
const char* miID = "JESIDI-IOT-POC-200574"; // Tu ID y Tópico
const char* mqtt_server = "broker.hivemq.com";

// --- OBJETOS ---
WiFiClient espClient;
PubSubClient client(espClient);
DHTesp dht;
UltrasonicSensor ultrasonic(13, 14);

// --- VARIABLES DE CONTROL ---
unsigned long lastMsg = 0;
unsigned long lastReconnectAttempt = 0;
int dhtPin = 15;
byte ledPins[] = {21, 47, 48, 38, 39, 40, 41, 42, 2, 1};
const int numLeds = 10;
const int maxDistancia = 300;

// --- FUNCIONES AUXILIARES ---
float getTemp() {
  float t = dht.getTemperature();
  return (isnan(t)) ? 20.0 : t;
}

void actualizarLeds(int distanciaCm) {
  int d = constrain(distanciaCm, 10, maxDistancia);
  int ledsAEncender = map(d, 0, maxDistancia, numLeds, 0);
  for (int i = 0; i < numLeds; i++) {
    digitalWrite(ledPins[i], (i < ledsAEncender) ? HIGH : LOW);
  }
}

boolean reconnect() {
  Serial.print("Intentando conexión MQTT...");
  // Intentamos conectar usando tu ID único
  if (client.connect(miID)) {
    Serial.println("¡CONECTADO AL BROKER!");
  } else {
    Serial.print("falló, rc=");
    Serial.print(client.state());
    Serial.println(" reintentando en 5 segundos");
  }
  return client.connected();
}

void setup() {
  // 1. Configuración Serial CRÍTICA para ESP32-S3
  Serial.begin(115200);
  
  Serial.println("\n**********************************");
  Serial.println("SISTEMA JESIDI-IOT INICIADO");
  Serial.println("**********************************");

  // 1. Configurar el botón físico (Pin 4)
  pinMode(4, INPUT_PULLUP); 

  // 2. WiFiManager (Si no hay red, crea el AP "ESP32_JESIDI")
  WiFiManager wm;
  
  // 2. Lógica de Reset Manual
  // Si el botón está pulsado (LOW) al encender la placa
  if (digitalRead(4) == LOW) {
    Serial.println(">>> MODO CONFIGURACIÓN ACTIVADO POR BOTÓN");
    wm.resetSettings(); // Opcional: borra si quieres empezar de cero
    wm.setConfigPortalTimeout(120);
  }

  if(!wm.autoConnect("ESP32_JESIDI")) {
    Serial.println("Fallo en la conexión. Reiniciando...");
    delay(3000);
    ESP.restart();
  }
  Serial.println("Wi-Fi conectado correctamente.");

  // 3. Configuración MQTT y Sensores
  client.setServer(mqtt_server, 1883);
  dht.setup(dhtPin, DHTesp::DHT11);
  
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  float tempInicial = getTemp();
  ultrasonic.setTemperature((int)tempInicial);
}

void loop() {
  // A. GESTIÓN DE CONEXIÓN (No bloqueante)
  if (!client.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }

  // B. LÓGICA DE SENSORES Y LEDS (Funciona siempre)
  int distance = ultrasonic.distanceInCentimeters();
  actualizarLeds(distance);

  // C. ENVÍO DE DATOS CADA 5 SEGUNDOS
  unsigned long now = millis();
  if (now - lastMsg > 500) {
    lastMsg = now;
    
    if (client.connected()) {
      float t = getTemp();
      float h = dht.getHumidity();

      // Si la humedad falla (nan), ponemos 0 o un valor de error para que no rompa el JSON
      if (isnan(h)) h = 0.0;
      if (isnan(t)) t = 0.0;
      delay(10);
      String payload = "{\"temp\":" + String(t,1) + ",\"hum\":" + String(h,1) + ",\"dist\":" + String(distance) + "}";
      
      client.publish(miID, payload.c_str());
      Serial.println("Publicado: " + payload);
    }
  }
  delay(50); // Estabilidad del loop
}
