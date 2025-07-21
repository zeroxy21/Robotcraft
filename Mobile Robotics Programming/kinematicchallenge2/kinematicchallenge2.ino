#include "math.h"

// Global variables
double x = 0, y = 0, teta = 0;
int N[10][2] = {{0,0}, {100,100}, {300,200}, {500,300}, {700,400}, 
               {900,500}, {1000,600}, {1100,700}, {1200,800}, {1200,800}};

// Function prototypes
void encUpdate(int t, int* NL, int* NR);
void poseUpdate(int NL, int NR, int r, int b, int C, double* x, double* y, double* theta);
void poseUpdate1(int NL, int NR, int r, int b, int C, double* x, double* y, double* theta, double* V, double* W);
void cmd_vel(double* V, double* W);
void cmd_vel2wheels(double V, double W, double* wl, double* wr);

void setup() {
  Serial.begin(9600);  
  int r = 50, b = 200, C = 100;
  
  for(int i = 1; i < 10; i++){ 
    int NL, NR;
    encUpdate(i, &NL, &NR); 
    
    double x1, y1, teta1, V, W;
    poseUpdate1(NL, NR, r, b, C, &x1, &y1, &teta1, &V, &W);
    
    x += x1;
    y += y1;
    teta += teta1;
    
    Serial.print("Position: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print("), Angle: ");
    Serial.println(teta);
  }
  
  delay(1000); 
}

void loop() {
  // You can add code here to run repeatedly
}

void encUpdate(int t, int* NL, int* NR) {
  *NL = N[t][0] - N[t-1][0];
  *NR = N[t][1] - N[t-1][1];
}

void poseUpdate(int NL, int NR, int r, int b, int C, double* x, double* y, double* theta) {
  double Dl, Dr;
  Dl = 2 * M_PI * r * NL / C;
  Dr = 2 * M_PI * r * NR / C;
  
  *theta = (Dr - Dl) / b;
  double Dm = (Dl + Dr) / 2;
  *x = Dm * cos(*theta);
  *y = Dm * sin(*theta);
}

void cmd_vel(double* V, double* W) {
    // Wait for serial input
    while (Serial.available() < 1) {
        delay(10);
    }
    *V = Serial.parseFloat();
    
    while (Serial.available() < 1) {
        delay(10);
    }
    *W = Serial.parseFloat();
    
    Serial.print("You entered linear and angular: ");
    Serial.print(*V);
    Serial.print(", ");
    Serial.println(*W);
}

void cmd_vel2wheels(double V, double W, double* wl, double* wr) {
  int r = 50, b = 200;
  *wl = (V - 0.5 * b * W) / r;
  *wr = (V + 0.5 * b * W) / r;
}

void poseUpdate1(int NL, int NR, int r, int b, int C, double* x, double* y, double* theta, double* V, double* W) {
  double Dl, Dr;
  Dl = 2 * M_PI * r * NL / C;
  Dr = 2 * M_PI * r * NR / C;
  
  *theta = (Dr - Dl) / b;
  double Dm = (Dl + Dr) / 2;
  *x = Dm * cos(*theta);
  *y = Dm * sin(*theta);
  *V = 2 * M_PI * r * (NR + NL) / (2 * C);
  *W = 2 * M_PI * r * (NR - NL) / (b * C);  // Fixed the formula for W
}
