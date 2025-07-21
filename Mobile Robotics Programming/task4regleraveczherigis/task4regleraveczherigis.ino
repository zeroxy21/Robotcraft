#include "math.h"

// Pins moteurs et encodeurs (identique à votre configuration originale)
#define M1R_DIR 5
#define M1R_PWM 4
#define M2L_DIR 6
#define M2L_PWM 9
#define ENC_M1R_A 18
#define ENC_M1R_B 19
#define ENC_M2L_A 3
#define ENC_M2L_B 2

// Variables encodeurs
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// Variables pour Ziegler-Nichols
unsigned long lastTime = 0;
const unsigned long sampleTime = 100; // 100ms
float Ku = 0; // Gain ultime à déterminer
float Tu = 0; // Période d'oscillation à déterminer
bool znTuningMode = false;
bool znStepApplied = false;
unsigned long znStartTime = 0;
float znStepValue = 0;
float znMaxSpeed = 0;
float znMinSpeed = 0;
int znOscillationCount = 0;
unsigned long znLastCrossTime = 0;

// Configuration minimale pour le test
float Kp = 0.1302f, Ki = 0.1554f, Kd = 0.0273f;
float previousErrorLeft = 0, previousErrorRight = 0;
float integralLeft = 0, integralRight = 0;

void setup() {
  Serial.begin(115200);
  
  // Configuration moteurs et encodeurs (identique)
  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);
  pinMode(ENC_M2L_A, INPUT);
  pinMode(ENC_M2L_B, INPUT);
  pinMode(ENC_M1R_A, INPUT);
  pinMode(ENC_M1R_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_M2L_A), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M1R_A), rightEncoder, CHANGE);
  
  Serial.println(F("System ready. Send 'z' to start Ziegler-Nichols tuning."));
}

void loop() {
  unsigned long currentTime = millis();
  
  // Démarrer le réglage Z-N
  if (Serial.available() && !znTuningMode) {
    if (Serial.read() == 'z') {
      startZieglerNichols();
    }
  }
  
  if (znTuningMode) {
    if (currentTime - lastTime >= sampleTime) {
      // Calcul vitesse
      float leftSpeed = (encoderLeftCount - lastLeftCount) * (1000.0 / sampleTime);
      lastLeftCount = encoderLeftCount;
      lastTime = currentTime;
      
      // Appliquer l'étape initiale
      if (!znStepApplied) {
        znStepApplied = true;
        znStartTime = currentTime;
        znStepValue = 50; // Valeur de l'étape (50 pulses/100ms)
        digitalWrite(M2L_DIR, LOW);
        analogWrite(M2L_PWM, 150); // Valeur PWM fixe pour le test
        digitalWrite(M1R_DIR, LOW);
        analogWrite(M1R_PWM, 150);
        Serial.println(F("Applying step input..."));
      }
      
      // Détection des oscillations
      detectOscillations(leftSpeed, currentTime);
      
      // Affichage des données
      Serial.print(F("T:"));
      Serial.print((currentTime - znStartTime)/1000.0, 1);
      Serial.print(F(",Speed:"));
      Serial.print(leftSpeed, 0);
      Serial.print(F(",Ku:"));
      Serial.print(Ku, 2);
      Serial.print(F(",Tu:"));
      Serial.println(Tu, 2);
      
      // Arrêt après 30 secondes max
      if (currentTime - znStartTime > 10000) {
        endZieglerNichols();
      }
    }
  }
  else {
    // Mode normal PID (non utilisé pendant le réglage)
    if (currentTime - lastTime >= sampleTime) {
      lastTime = currentTime;
      // Ici vous pourriez implémenter le contrôle PID normal
    }
  }
}

void startZieglerNichols() {
  znTuningMode = true;
  znStepApplied = false;
  znOscillationCount = 0;
  Ku = 0;
  Tu = 0;
  znMaxSpeed = 0;
  znMinSpeed = 2000; // Valeur initiale haute
  Serial.println(F("Starting Ziegler-Nichols tuning..."));
}

void endZieglerNichols() {
  znTuningMode = false;
  analogWrite(M2L_PWM, 0);
  analogWrite(M1R_PWM, 0);
  
  // Calcul des paramètres PID selon Ziegler-Nichols
  if (Tu > 0 && Ku > 0) {
    calculatePIDParameters();
    Serial.println(F("Ziegler-Nichols results:"));
    Serial.print(F("Ku (Ultimate Gain): "));
    Serial.println(Ku, 2);
    Serial.print(F("Tu (Oscillation Period): "));
    Serial.println(Tu, 2);
    Serial.print(F("Calculated Kp: "));
    Serial.println(Kp, 4);
    Serial.print(F("Calculated Ki: "));
    Serial.println(Ki, 4);
    Serial.print(F("Calculated Kd: "));
    Serial.println(Kd, 4);
  } else {
    Serial.println(F("Failed to determine Ku and Tu. Try again."));
  }
}

void detectOscillations(float speed, unsigned long currentTime) {
  // Mise à jour des min/max
  if (speed > znMaxSpeed) znMaxSpeed = speed;
  if (speed < znMinSpeed) znMinSpeed = speed;
  
  // Détection du passage par la moyenne
  float avgSpeed = (znMaxSpeed + znMinSpeed) / 2;
  static bool wasAbove = false;
  bool isAbove = speed > avgSpeed;
  
  if (isAbove != wasAbove) {
    // Croisement détecté
    if (znLastCrossTime > 0) {
      float period = (currentTime - znLastCrossTime) / 1000.0; // en secondes
      Tu = (Tu * znOscillationCount + period) / (znOscillationCount + 1);
    }
    znLastCrossTime = currentTime;
    znOscillationCount++;
    
    // Calcul de Ku (approximation)
    if (znOscillationCount >= 2) {
      Ku = (4 * 150) / (M_PI * (znMaxSpeed - znMinSpeed)); // 150 = valeur PWM appliquée
    }
  }
  wasAbove = isAbove;
}

void calculatePIDParameters() {
  // Méthode de Ziegler-Nichols pour un régulateur PID
  Kp = 0.6 * Ku;
  Ki = 1.2 * Ku / Tu;
  Kd = 0.075 * Ku * Tu;
  
  // Ajustement pour notre échantillonnage de 100ms
  Ki *= (sampleTime / 1000.0); // Intégrale devient somme des erreurs * Ki * dt
  Kd /= (sampleTime / 1000.0); // Dérivée devient différence d'erreur / dt
}

// Fonctions encodeurs (inchangées)
void leftEncoder() {
  encoderLeftCount += digitalRead(ENC_M2L_B) ? -1 : 1;
}

void rightEncoder() {
  encoderRightCount += digitalRead(ENC_M1R_B) ? -1 : 1;
}