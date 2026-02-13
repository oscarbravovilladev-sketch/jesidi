#include <UltrasonicSensor.h>

#define PIN_ANALOG_IN 4
UltrasonicSensor ultrasonic(13, 14);

byte ledPins[] = {21, 47, 48, 38, 39, 40, 41, 42, 2, 1};
const int numLeds = 10;
const int maxDistancia = 300; // cm

void setup() {
  Serial.begin(115200);
  
  // Configuración de pines de salida
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  // Medición inicial de temperatura para calibrar el sensor
  float tempInicial = getTemp();
  ultrasonic.setTemperature(tempInicial);
  
  // CORRECCIÓN: Usar %.2f para mostrar 2 decimales
  Serial.printf("Temperatura inicial: %.2f C\n", tempInicial);
}

void loop() {
  // 1. Obtener temperatura actual y actualizar el sensor
  float currentTemp = getTemp();
  ultrasonic.setTemperature(currentTemp);

  // 2. Medir distancia
  int distance = ultrasonic.distanceInCentimeters();

  // 3. Actualizar LEDs (Inversamente: - distancia = + leds)
  actualizarLeds(distance);

  // 4. Mostrar datos por Serial
  Serial.printf("Temp: %.2f C | Distancia: %d cm\n", currentTemp, distance);
  
  delay(300); // Reducido para mayor fluidez
}

double getTemp(){
  int adcValue = analogRead(PIN_ANALOG_IN); 
  // Nota: ESP32 usa 12 bits (4095) por defecto
  double voltage = (double)adcValue / 4095.0 * 3.3; 
  double Rt = 10 * voltage / (3.3 - voltage); 
  double tempK = 1 / (1/(273.15 + 25) + log(Rt / 10)/3950.0);
  return tempK - 273.15; 
}

void actualizarLeds(int distanciaCm) {
  // Clamp: Limita entre 0 y el máximo definido
  int d = constrain(distanciaCm, 0, maxDistancia);

  // Mapeo Inverso: 
  // 0cm -> numLeds (10)
  // maxDistancia -> 0 leds
  int ledsAEncender = map(d, 0, maxDistancia, numLeds, 0);

  for (int i = 0; i < numLeds; i++) {
    digitalWrite(ledPins[i], (i < ledsAEncender) ? HIGH : LOW);
  }
}
