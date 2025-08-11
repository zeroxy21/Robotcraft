#include <Encoder.h>
#include <TimerOne.h>

// === MOTEURS ===
#define M1R_PWM 4
#define M1R_DIR 5
#define M2L_PWM 9
#define M2L_DIR 6

// === ENCODEURS ===
#define ENC_M1R_A 3
#define ENC_M1R_B 2
#define ENC_M2L_A 19
#define ENC_M2L_B 18

Encoder enc_right(ENC_M1R_A, ENC_M1R_B);
Encoder enc_left(ENC_M2L_A, ENC_M2L_B);

// === PARAMÈTRES ===
float C = 0.00241;  // ticks → cm

// === CONSIGNES ===
float vd = 10.0;  // cm/s - droite
float vg = 10.0;  // cm/s - gauche

// === VARIABLES PID DROITE ===
float Kp_r = 1.2, Ki_r = 5.88, Kd_r = 0.061;
float error_r = 0, last_error_r = 0;
float integral_r = 0, derivative_r = 0;
float output_r = 0;
float vitesse_r = 0;

// === VARIABLES PID GAUCHE ===
float Kp_l = 1.2, Ki_l = 5.88, Kd_l = 0.061;
float error_l = 0, last_error_l = 0;
float integral_l = 0, derivative_l = 0;
float output_l = 0;
float vitesse_l = 0;

// === ENCODEURS ===
long last_enc_r = 0;
long last_enc_l = 0;

// === FLAG TIMER ===
volatile bool flag = false;

void setup() {
  Serial.begin(9600);

  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);

  last_enc_r = enc_right.read();
  last_enc_l = enc_left.read();

  Timer1.initialize(100000);  // 100 ms
  Timer1.attachInterrupt(callback);
}

void loop() {
  if (flag) {
    flag = false;

    // === PID DROITE ===
    error_r = vd - vitesse_r;
    integral_r += error_r * 0.1;
    derivative_r = (error_r - last_error_r) / 0.1;
    last_error_r = error_r;

    output_r = Kp_r * error_r + Ki_r * integral_r + Kd_r * derivative_r;
    output_r = constrain(output_r, -255, 255);

    if (output_r >= 0) {
      digitalWrite(M1R_DIR, LOW);
    } else {
      digitalWrite(M1R_DIR, HIGH);
    }
    analogWrite(M1R_PWM, abs(output_r));

    // === PID GAUCHE ===
    error_l = vg - vitesse_l;
    integral_l += error_l * 0.1;
    derivative_l = (error_l - last_error_l) / 0.1;
    last_error_l = error_l;

    output_l = Kp_l * error_l + Ki_l * integral_l + Kd_l * derivative_l;
    output_l = constrain(output_l, -255, 255);
    
    if (output_l >= 0) {
      digitalWrite(M2L_DIR, LOW);
    } else {
      digitalWrite(M2L_DIR, HIGH);
    }
    analogWrite(M2L_PWM, abs(output_l));

    // === DEBUG SERIAL ===
    Serial.print("VR: "); Serial.print(vitesse_r);
    Serial.print(" | VL: "); Serial.print(vitesse_l);
    Serial.print(" | OR: "); Serial.print(output_r);
    Serial.print(" | OL: "); Serial.println(output_l);
  }
}

void callback() {
  long current_r = enc_right.read();
  long delta_r = current_r - last_enc_r;
  last_enc_r = current_r;
  vitesse_r = (C * delta_r) / 0.1;

  long current_l = enc_left.read();
  long delta_l = current_l - last_enc_l;
  last_enc_l = current_l;
  vitesse_l = (C * delta_l) / 0.1;

  flag = true;
}

