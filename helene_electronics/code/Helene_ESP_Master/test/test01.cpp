#include <Arduino.h>
#include <AS5048A.h>

#define PIN_CS_EXT_AMS 18

AS5048A obj_angleSensor(PIN_CS_EXT_AMS);
unsigned long last_print = 0;

void setup() {
  // Impostiamo la seriale a 1.000.000 così leggerai il testo chiaro senza toccare il monitor
  Serial.begin(1000000);
  delay(2000);
  Serial.println("\n--- TEST ENCODER MAGNETICO ---");

  obj_angleSensor.init();
  Serial.println("Sensore AS5048A inizializzato. Muovi il giunto a mano!");
}

void loop() {
  if (millis() - last_print > 150) {
    last_print = millis();
    
    uint16_t raw_angle = obj_angleSensor.getRawRotation();
    long current_angle = obj_angleSensor.getRotation();

    Serial.print("Grezzo: ");
    Serial.print(raw_angle);
    Serial.print(" | Angolo Gradi: ");
    Serial.println(current_angle);
  }
}