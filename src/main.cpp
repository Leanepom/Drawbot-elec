#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Définition des broches
#define EN_D 23
#define EN_G 4

#define IN_1_D 19
#define IN_2_D 18

#define IN_1_G 17
#define IN_2_G 16

// Fonctions de mouvement
void avancer() {
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, HIGH);
  digitalWrite(IN_1_G, HIGH);
  digitalWrite(IN_2_G, LOW);
}

void tournerGauche() {
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, HIGH);
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, HIGH);
}

void tournerDroite() {
  digitalWrite(IN_1_D, HIGH);
  digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, HIGH);
  digitalWrite(IN_2_G, LOW);
}

void reculer() {
  digitalWrite(IN_1_D, HIGH);
  digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, HIGH);
}

void stopMotors() {
  digitalWrite(IN_1_D, LOW);
  digitalWrite(IN_2_D, LOW);
  digitalWrite(IN_1_G, LOW);
  digitalWrite(IN_2_G, LOW);
}

// Fonction pour dessiner la flèche
void sequenceFleche() {
  // À ajuster expérimentalement selon la vitesse des moteurs
  int avance_12v5cm = 312.5;  // Durée en ms pour 12.5 cm
  int recule_12v5cm = 312.5;  // Durée pour 12.5 cm
  int tourne_gauche_45 = 100; // Durée pour 45° de rotation
  int tourne_droite_90 = 200; // Durée pour 90° de rotation
  int avance_20cm = 500;  // Durée pour 20 cm

  // Avancer de 12.5 cm
  avancer();
  delay(avance_12v5cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a avancé de 10cm.");

  // Tourner 45° à gauche
  tournerGauche();
  delay(tourne_gauche_45);
  stopMotors();
  delay(500);
  SerialBT.println("Il a tourné à gauche.");

  // Reculer de 12.5 cm
  reculer();
  delay(recule_12v5cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a reculé de 10cm.");
  
  // Avancer de 12.5 cm
  avancer();
  delay(avance_12v5cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a avancé de 10cm.");

  // Tourner 90° à droite
  tournerDroite();
  delay(tourne_droite_90);
  stopMotors();
  delay(500);
  SerialBT.println("Il a tourné à droite.");

  // Reculer de 12.5 cm
  reculer();
  delay(recule_12v5cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a reculé de 10cm.");
  
  // Avancer de 12.5 cm
  avancer();
  delay(avance_12v5cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a avancé de 10cm.");

  // Tourner 45° à gauche
  tournerGauche();
  delay(tourne_gauche_45);
  stopMotors();
  delay(500);
  SerialBT.println("Il a tourné à gauche.");

  // Avancer de 20 cm
  avancer();
  delay(avance_20cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a avancé de 20 cm.");

  SerialBT.println("Séquence flèche terminée !");
}

// Fonction pour exécuter l’escalier
void sequenceEscalier() {
  // À ajuster expérimentalement selon la vitesse des moteurs
  int avance_20cm = 2000;  // Durée en ms pour 20 cm
  int avance_10cm = 1000;  // Durée pour 10 cm
  int avance_40cm = 4000;  // Durée pour 40 cm
  int rotation_90 = 200;   // Durée pour 90° de rotation (ajuster selon tests)

  // Avancer de 20 cm
  avancer();
  delay(avance_20cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a avancé de 20cm.");

  // Tourner 90° à gauche
  tournerGauche();
  delay(rotation_90);
  stopMotors();
  delay(500);
  SerialBT.println("Il a tourné à gauche.");

  // Avancer de 10 cm
  avancer();
  delay(avance_10cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a avancé de 10cm.");

  // Tourner 90° à droite
  tournerDroite();
  delay(rotation_90);
  stopMotors();
  delay(500);
  SerialBT.println("Il a tourné à droite.");

  // Avancer de 40 cm
  avancer();
  delay(avance_40cm);
  stopMotors();
  delay(500);
  SerialBT.println("Il a avancé de 40 cm.");

  SerialBT.println("Séquence escalier terminée !");
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_ROBOT_BT");

  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);

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
      case '1': 
        tournerDroite();
        SerialBT.println("Tourner à droite");
        break;
      case '2': 
        tournerGauche();
        SerialBT.println("Tourner à gauche");
        break;
      case '3': 
        reculer();
        SerialBT.println("Reculer");
        break;
      case '4': 
        avancer();
        SerialBT.println("Avancer");
        break;
        case '5': // Séquence escalier
          SerialBT.println("Début séquence escalier...");
          sequenceEscalier();
          break;
      case '6': // Séquence flèche
        SerialBT.println("Début séquence flèche...");
        sequenceFleche();
        break;
      case '0': 
        stopMotors();
        SerialBT.println("Stop");
        break;
      default:
        SerialBT.println("Commande inconnue");
        break;
    }
  }
}