#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Définition des broches (basé sur ton image)
#define EN_D 23
#define EN_G 4

#define IN_1_D 19
#define IN_2_D 18

#define IN_1_G 17
#define IN_2_G 16

// Fonctions de mouvement
void tournerDroite() {
  digitalWrite(IN_1_D, HIGH);
  digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, HIGH);
  digitalWrite(IN_2_G, LOW);
}

void reculer() {
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, HIGH);
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, HIGH);
}

void tournerGauche() {
  digitalWrite(IN_1_D, HIGH);
  digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, HIGH);
}

void avancer() {
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, HIGH);
  digitalWrite(IN_1_G, HIGH);
  digitalWrite(IN_2_G, LOW);
}

void stopMotors() {
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, LOW);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_ROBOT_BT"); // Nom du module Bluetooth

  // Configuration des broches moteurs
  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);

  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);

  // Activation des moteurs au démarrage
  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  stopMotors();

  Serial.println("Bluetooth actif. Attente de commande...");
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.print("Commande reçue : ");
    Serial.println(cmd);

    switch (cmd) {
      case '1': // Tourner à droite
        tournerDroite();
        SerialBT.println("Tourner à droite");
        break;
      case '2': // Tourner à gauche
        reculer();
        SerialBT.println("Tourner à gauche");
        break;
      case '3': // Reculer
        tournerGauche();
        SerialBT.println("Reculer");
        break;
      case '4': // Avancer
        avancer();
        SerialBT.println("Avancer");
        break;
      case '0': // Stop
        stopMotors();
        SerialBT.println("Stop");
        break;
      default:
        SerialBT.println("Commande inconnue");
        break;
    }
  }
}
