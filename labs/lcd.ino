#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#define SDA 14
#define SCL 13
LiquidCrystal_I2C lcd(0x27, 16, 2);
void setup() {
  // SIN Serial.begin() - No usar Serial Monitor
  
  Wire.begin(SDA, SCL);
  Wire.setClock(50000);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("Test SIN Serial");
}
void loop() {
  lcd.setCursor(0, 1);
  lcd.print("Tiempo: ");
  lcd.print(millis() / 1000);
  lcd.print(" s   ");
  
  delay(500);
}