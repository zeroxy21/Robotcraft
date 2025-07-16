
#include "math.h"
#include "encoder.h"
// define motors pins
#define M1R_DIR 5  // RIGHT motor direction | Set pin to LOW to move front ! HIGH to move back
#define M1R_PWM 4 // Set 0 to 255 to set RIGHT motor speed
#define M2L_DIR 6 // LEFT motor direction  | Set pin to LOW to move front ! HIGH to move back
#define M2L_PWM 9 // Set 0 to 255 to set LEFT motor speed
// Encoders Pins
#define ENC_M1R_A 18  // RIGHT encoder channel A - C1
#define ENC_M1R_B 19  // RIGHT encoder channel B - C2
#define ENC_M2L_A 3 // LEFT encoder channel A - C1
#define ENC_M2L_B 2 // LEFT encoder channel B - C2

// Variables encodeurs
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// Variables PID
unsigned long lastTime = 0;
float Kp = 0.5, Ki = 0, Kd = 0;
float previousErrorLeft = 0, previousErrorRight = 0;
float integralLeft = 0, integralRight = 0;

void setup() {
  Serial.begin(115200);
  
  // Configuration moteurs
  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);
  
  // Configuration encodeurs + interruptions
  pinMode(ENC_M2L_A, INPUT);
  pinMode(ENC_M2L_B, INPUT);
  pinMode(ENC_M1R_A, INPUT);
  pinMode(ENC_M1R_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_M2L_A), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M1R_A), rightEncoder, CHANGE);
  
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastTime >= 100) {  // 100ms d'échantillonnage
    // Calcul vitesse
    float leftSpeed = encoderLeftCount - lastLeftCount;
    float rightSpeed = encoderRightCount - lastRightCount;
    
    // Réinitialisation compteurs
    lastLeftCount = encoderLeftCount;
    lastRightCount = encoderRightCount;
    lastTime = currentTime;
    
    // Consigne (exemple)
    float desiredSpeed = 100.0;
    
    // PID pour chaque moteur
    float leftOutput = pid_controller(desiredSpeed, leftSpeed, &integralLeft, &previousErrorLeft);
    float rightOutput = pid_controller(desiredSpeed, rightSpeed, &integralRight, &previousErrorRight);
    
    // Commande moteurs
    digitalWrite(M2L_DIR, LOW);  // Sens avant
    analogWrite(M2L_PWM, constrain(leftOutput, 0, 255));
    
    digitalWrite(M1R_DIR, LOW);  // Sens avant
    analogWrite(M1R_PWM, constrain(rightOutput, 0, 255));
    
    // Debug Serial
    Serial.print("Desired:");
    Serial.print(desiredSpeed);
    Serial.print(",Left:");
    Serial.print(leftSpeed);
    Serial.print(",Right:");
    Serial.println(rightSpeed);
  }
}

// Interruption encodeur GAUCHE
void leftEncoder() {
  if (digitalRead(ENC_M2L_B)) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
}

// Interruption encodeur DROIT
void rightEncoder() {
  if (digitalRead(ENC_M1R_B)) {
    encoderRightCount--;
  } else {
    encoderRightCount++;
  }
}

// Fonction PID (identique à ton original)
float pid_controller(float desiredSpeed, float realSpeed, float* integral, float* previousError) {
  float error = desiredSpeed - realSpeed;
  
  *integral += error;
  *integral = constrain(*integral, -255/Ki, 255/Ki);  // Anti-windup
  
  float derivative = error - *previousError;
  float output = Kp * error + Ki * (*integral) + Kd * derivative;
  
  *previousError = error;
  
  return output;
}
