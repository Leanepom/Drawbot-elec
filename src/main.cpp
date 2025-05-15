#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// Broches moteurs
#define EN_D 23
#define EN_G 4
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16

// Broches encodeurs
#define ENC_D_A 27
#define ENC_G_A 32

// Variables encodeurs
volatile long ticks_droit = 0;
volatile long ticks_gauche = 0;

// Paramètres physiques calibrés
const float TICKS_PAR_TOUR = 830.0;   // Calibré à partir des mesures réelles
const float DIAMETRE_ROUE_CM = 9.0;   // 90 mm
const float EMPATTEMENT_CM = 13.0;    // Ajusté

// PID pour correction de trajectoire
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float previous_error = 0;
float integral = 0;

// PWM par défaut
int pwm_normal = 120;
int pwm_lent = 100;

// Fonctions moteurs
void setMotorPWM(int pwm_gauche, int pwm_droit) {
  pwm_gauche = constrain(pwm_gauche, -255, 255);
  pwm_droit = constrain(pwm_droit, -255, 255);

  if (pwm_gauche >= 0) {
    analogWrite(IN_1_G, pwm_gauche);
    analogWrite(IN_2_G, 0);
  } else {
    analogWrite(IN_1_G, 0);
    analogWrite(IN_2_G, -pwm_gauche);
  }

  if (pwm_droit >= 0) {
    analogWrite(IN_1_D, 0);
    analogWrite(IN_2_D, pwm_droit);
  } else {
    analogWrite(IN_1_D, -pwm_droit);
    analogWrite(IN_2_D, 0);
  }
}

void stopMotors() {
  setMotorPWM(0, 0);
}

// ISR encodeurs
void IRAM_ATTR onTickDroit() { ticks_droit++; }
void IRAM_ATTR onTickGauche() { ticks_gauche++; }

// Conversion distance -> ticks
long distanceToTicks(float distance_cm) {
  float perimetre = PI * DIAMETRE_ROUE_CM;
  return (distance_cm / perimetre) * TICKS_PAR_TOUR;
}

// Avancer avec PID (sans facteur de correction exagéré)
void avancerDistance(float distance_cm) {
  ticks_droit = 0;
  ticks_gauche = 0;
  previous_error = 0;
  integral = 0;

  int base_pwm = pwm_normal;
  long target_ticks = distanceToTicks(distance_cm);

  while ((ticks_droit + ticks_gauche) / 2 < target_ticks) {
    long erreur = ticks_gauche - ticks_droit;

    integral += erreur;
    float correction = Kp * erreur + Ki * integral + Kd * (erreur - previous_error);
    previous_error = erreur;

    int pwm_gauche = base_pwm - correction;
    int pwm_droit = base_pwm + correction;

    setMotorPWM(pwm_gauche, pwm_droit);
    delay(10);
  }

  setMotorPWM(base_pwm / 2, base_pwm / 2);
  delay(100);

  stopMotors();
  delay(300);
}

// Reculer avec PID
void reculerDistance(float distance_cm) {
  ticks_droit = 0;
  ticks_gauche = 0;
  previous_error = 0;
  integral = 0;

  int base_pwm = pwm_normal;
  long target_ticks = distanceToTicks(distance_cm);

  while ((ticks_droit + ticks_gauche) / 2 < target_ticks) {
    long erreur = ticks_droit - ticks_gauche;

    integral += erreur;
    float correction = Kp * erreur + Ki * integral + Kd * (erreur - previous_error);
    previous_error = erreur;

    // PWM négatif pour reculer
    int pwm_gauche = -(base_pwm + correction);
    int pwm_droit  = -(base_pwm - correction);

    setMotorPWM(pwm_gauche, pwm_droit);
    delay(10);
  }

  setMotorPWM(-base_pwm / 2, -base_pwm / 2);
  delay(100);

  stopMotors();
  delay(300);
}

// Rotation
void tournerAngle(float angle_deg, int base_pwm, bool gauche) {
  ticks_droit = 0;
  ticks_gauche = 0;

  float distance_par_roue = (PI * EMPATTEMENT_CM * angle_deg) / 360.0;
  long target_ticks = distanceToTicks(distance_par_roue);

  while (max(ticks_droit, ticks_gauche) < target_ticks) {
    if (gauche) {
      setMotorPWM(-base_pwm, base_pwm);
    } else {
      setMotorPWM(base_pwm, -base_pwm);
    }
    delay(10);
  }

  stopMotors();
  delay(500);
}

void symbole(float angle_deg, int rayon_cm, bool sensHoraire) {
  // Avance une roue pendant que l’autre reste fixe pour dessiner un arc
  ticks_droit = 0;
  ticks_gauche = 0;

  // Calcul de la distance que doit parcourir une roue sur un arc de cercle
  float distance_arc = 2 * PI * rayon_cm * (angle_deg / 360.0);
  long ticks_arc = distanceToTicks(distance_arc);

  while (max(ticks_droit, ticks_gauche) < ticks_arc) {
    if (sensHoraire) {
      // Roue gauche avance, roue droite fixe
      setMotorPWM(pwm_lent, 0);
    } else {
      // Roue droite avance, roue gauche fixe
      setMotorPWM(0, pwm_lent);
    }
    delay(10);
  }

  stopMotors();
  delay(300);
}

// Sequence escalier
void sequenceEscalier() {
  SerialBT.println("Début de la séquence escalier...");

  symbole(44.0, 8, true);  // Marque de départ (sens horaire)
  symbole(22.0,8,false); //Suite marque de départ 

  avancerDistance(17.0); // ajusté pour 20 cm réel  
  tournerAngle(65.0, pwm_normal, true); // correction d'angle pour 90° théorique
  avancerDistance(5.5); // ajusté pour 10 cm réel   
  tournerAngle(65.0, pwm_normal, false); // correction d'angle pour 90° théorique
  avancerDistance(42.5); // ajusté pour 40 cm réel 

  symbole(22.0, 8, false); // Marque d’arrivée (sens anti-horaire)
  symbole(44.0 , 8, true); //Suite marque d'arrivée 

  SerialBT.println("Séquence de l'escalier terminée !");
}

//Séquence escargot avec paramètres 
void sequenceEscargot(float longueur_initiale, int nbTours) {
  SerialBT.println("Début de la séquence escargot (paramétrée)...");

  float facteurCorrection = 3.0 / 6.5;   
  float angleCorrige = 85.0;

  float longueur = longueur_initiale * facteurCorrection;
  float increment = 4.0 * facteurCorrection;

  for (int i = 0; i < nbTours * 2; i++) {
    avancerDistance(longueur);
    tournerAngle(angleCorrige, pwm_normal, true);
    if (i % 2 == 1) {
      longueur += increment;
    }
  }

  SerialBT.println("Séquence escargot terminée !");
}

void cercle(float rayonStylo) {
  float distanceStylo = 12.5; // Distance entre roues et stylo
  float demiEmpattement = 4.5;

  float rayonCentre = rayonStylo - distanceStylo;
  if (rayonCentre <= 0) {
    SerialBT.println("Rayon trop petit pour exécuter un cercle valide.");
    return;
  }

  float rayonInterieur = rayonCentre - demiEmpattement;
  float rayonExterieur = rayonCentre + demiEmpattement;

  float rapportVitesse = rayonInterieur / rayonExterieur;

  int pwm_droit = pwm_normal;
  int pwm_gauche = (int)(pwm_droit * rapportVitesse);
  if (pwm_gauche < 1) pwm_gauche = 1;

  float distance_par_tick = (PI * DIAMETRE_ROUE_CM) / TICKS_PAR_TOUR;

  // On veut que le robot tourne de 360°
  float angle_vise_rad = 2 * PI;

  ticks_droit = 0;
  ticks_gauche = 0;

  SerialBT.println("Début cercle (par angle)...");

  while (true) {
    // Angle tourné = différence de distance entre les roues divisé par empattement
    float angle_actuel = (abs(ticks_droit - ticks_gauche) * distance_par_tick) / EMPATTEMENT_CM;

    if (angle_actuel >= angle_vise_rad) break;

    setMotorPWM(pwm_gauche, pwm_droit);
    delay(10);
  }

  stopMotors();
  SerialBT.println("Cercle terminé !");
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

  pinMode(ENC_D_A, INPUT_PULLUP);
  pinMode(ENC_G_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), onTickDroit, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_G_A), onTickGauche, RISING);

  stopMotors();
  Serial.println("Bluetooth actif. En attente de commande...");
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    Serial.print("Commande recue : ");
    Serial.println(cmd);

    switch (cmd)
    {
    case 'a':
      sequenceEscalier();
      break;
    case 'b':
    {
      SerialBT.println("Saisir la longueur initiale (cm) :");
    while (!SerialBT.available());
    float longueurInitiale = SerialBT.parseFloat();
    SerialBT.println(longueurInitiale);

    SerialBT.println("Saisir le nombre de tours :");
    while (!SerialBT.available());
    int nbTours = SerialBT.parseInt();
    SerialBT.println(nbTours);

    sequenceEscargot(longueurInitiale, nbTours);
    break;
    }
    case 'c':
      cercle(18.0);
      break; 
    case 'd': 
      //sequenceRosace(); 
      break;
    case 'e': 
      //sequenceFlèche(); 
      break; 
    case 'f': 
      //sequenceRosedesvents(); 
      break;  
    case 's':
      stopMotors();
      SerialBT.println("Stop");
      default:
      break;
    }
  }
}

