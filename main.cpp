
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT); // LED intégrée
}

void loop() {
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);
}