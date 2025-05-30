#include "BluetoothSerial.h"
#include <Wire.h>
//#include <Adafruit_LIS3MDL.h>
//#include <Adafruit_Sensor.h>
#include <math.h>

BluetoothSerial SerialBT;

// Moteur - ESP32
#define EN_D 23
#define EN_G 4
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16

// Encodeurs
#define ENC_D_A 27
#define ENC_G_A 32

// Paramètres physiques
const float DIAMETRE_ROUE_CM = 9.0;
const float EMPATTEMENT_CM = 8.9;
const float TICKS_PAR_TOUR = 830.0;
const float PERIMETRE_ROUE = PI * DIAMETRE_ROUE_CM;

// PID pour les distances
float Kp = 4.0, Ki = 0.05, Kd = 0.1;
float previous_error = 0;
float integral = 0;

// PWM
int pwm_normal = 120;

// Position
float posX = 0.0, posY = 0.0, angleRobot = 0.0;

// Correction distance
const float facteur_echelle = 0.035;  // Ajusté depuis 23.5 → allonge un peu la distance

// Encodeurs
volatile long ticks_droit = 0;
volatile long ticks_gauche = 0;

void IRAM_ATTR onTickDroit() { ticks_droit++; }
void IRAM_ATTR onTickGauche() { ticks_gauche++; }

void setupPWM() {
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);
  ledcSetup(2, 1000, 8);
  ledcSetup(3, 1000, 8);
  ledcAttachPin(IN_1_D, 0);
  ledcAttachPin(IN_2_D, 1);
  ledcAttachPin(IN_1_G, 2);
  ledcAttachPin(IN_2_G, 3);
}

void setMotorPWM(int pwm_gauche, int pwm_droit) {
  pwm_gauche = constrain(pwm_gauche, -255, 255);
  pwm_droit = constrain(pwm_droit, -255, 255);
  if (pwm_gauche >= 0) {
    ledcWrite(2, pwm_gauche);
    ledcWrite(3, 0);
  } else {
    ledcWrite(2, 0);
    ledcWrite(3, -pwm_gauche);
  }
  if (pwm_droit >= 0) {
    ledcWrite(0, 0);
    ledcWrite(1, pwm_droit);
  } else {
    ledcWrite(0, -pwm_droit);
    ledcWrite(1, 0);
  }
}

void stopMotors() {
  setMotorPWM(0, 0);
  delay(200);
}

long distanceToTicks(float distance_cm) {
  return (distance_cm / PERIMETRE_ROUE) * TICKS_PAR_TOUR;
}

void avancerVers(float x_target, float y_target) {
  float dx = x_target - posX;
  float dy = y_target - posY;
  float target_angle = atan2(dy, dx);
  float distance = sqrt(dx * dx + dy * dy) * facteur_echelle;

  float travelled = 0;
  float step = 0.1;

  while (travelled < distance) {
    float error = 0;
    integral += error;
    float derivative = error - previous_error;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    int pwm_gauche = pwm_normal - correction;
    int pwm_droit = pwm_normal + correction;
    setMotorPWM(pwm_gauche, pwm_droit);

    delay(100);
    posX += step * cos(target_angle);
    posY += step * sin(target_angle);
    travelled += step;
  }

  stopMotors();
  angleRobot = target_angle;
}


void sequenceEscalier() {
  SerialBT.println("Démarrage escalier...");
  posX = 0.0; posY = 0.0; angleRobot = 0.0;
  ticks_droit = 0; ticks_gauche = 0; // reset encodeurs

  // 1. Ligne droite 20 cm
  avancerVers(20.0, 0.0);
  SerialBT.println("Fin ligne 1");

  // 2. Amorce (rotation sur place)
  setMotorPWM(-100, 100);
  delay(145);
  stopMotors();
  delay(300);
  SerialBT.println("Fin amorce");

  // 3. Courbe gauche avec PID sur différence ticks (roue droite vs gauche)
  int base_pwm_gauche = 70;
  int pwm_droite = 370;
  int duration_ms = 135; // durée totale du virage
  int step_delay = 20;
  int steps = duration_ms / step_delay;

  float pid_Kp = 2.0; // PID pour équilibrer les ticks (pas l’angle réel)
  float pid_Ki = 0.0;
  float pid_Kd = 0.1;
  float pid_integral = 0;
  float pid_prev_error = 0;

  for (int k = 0; k < steps; k++) {
    long diff_ticks = ticks_droit - ticks_gauche;
    float error = -diff_ticks; // objectif : égaliser avance des roues
    pid_integral += error;
    float derivative = error - pid_prev_error;
    float correction = pid_Kp * error + pid_Ki * pid_integral + pid_Kd * derivative;
    pid_prev_error = error;

    int pwm_gauche_corrige = constrain(base_pwm_gauche + correction, 50, 255);
    setMotorPWM(pwm_gauche_corrige, pwm_droite);
    delay(step_delay);
  }

  stopMotors();
  SerialBT.println("Fin courbe gauche");

  // 4. Courbe droite (inchangée)
  int j = 70;
  while (j < 205) {
    setMotorPWM(205, j);
    j += 3;
    delay(20);
  }
  SerialBT.println("Fin courbe droite");

  // Correction finale pour compléter les 7 cm manquants
  float dx = 4.3 * cos(angleRobot);
  float dy = 4.3 * sin(angleRobot);
  avancerVers(posX + dx, posY + dy);
  SerialBT.println("Correction 7 cm ajoutée");

  SerialBT.println("Escalier terminé");
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_ROBOT_BT");

  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);

  pinMode(ENC_D_A, INPUT_PULLUP);
  pinMode(ENC_G_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), onTickDroit, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_G_A), onTickGauche, RISING);

  setupPWM();
  stopMotors();
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    if (cmd == 'a') {
      sequenceEscalier();
    }
    if (cmd == 'b') {
      //sequenceSpirale();
    }
    if (cmd == 'c') {
      //sequenceCercle();
    }
    if (cmd == 'd') {
      //sequenceRosace();
    }
    if (cmd == 'e') {
      //sequenceFleche();
    }
    if (cmd == 'f') {
      //sequenceRosedesvents();
    }
  }
}