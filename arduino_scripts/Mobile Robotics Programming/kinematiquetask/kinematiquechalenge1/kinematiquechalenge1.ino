#include "math.h"

// Global variables
double x = 0, y = 0, teta = 0;
int N[10][2] = {{0,0}, {100,100}, {300,200}, {500,300}, {700,400}, 
               {900,500}, {1000,600}, {1100,700}, {1200,800}, {1200,800}};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Initialize serial communication for debugging
   int r = 50, b = 200, C = 100;
  
  for(int i = 1; i < 10; i++){  // Start from 1 since we're comparing with previous
    int NL, NR;
    encUpdate(i, &NL, &NR);  // Pass pointers to get both return values
    
    double x1, y1, teta1;
    poseUpdate(NL, NR, r, b, C, &x1, &y1, &teta1);
    
    x += x1;
    y += y1;
    teta += teta1;
    
    // Print results for debugging
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
  // put your main code here, to run repeatedly:
  // Delay to prevent flooding the serial monitor
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
