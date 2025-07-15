#define FLEX_SENSOR_PIN A5  // Broche où est connecté le capteur de flexion
#define LED_PIN 13


void setup() {
    Serial.begin(9600);  // Initialisation 
    pinMode(FLEX_SENSOR_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    int flexValue = analogRead(FLEX_SENSOR_PIN);  // Lire la valeur du capteur
    float voltage = flexValue * (5.0 / 1023.0);   // Convertir en tension (0 à 5V)
    Serial.print("Valeur ADC : ");
    Serial.print(flexValue);
    Serial.print(" | Tension (V) : ");
    Serial.println(voltage, 2); // Affichage avec 2 décimales
    if (flexValue>700){
      digitalWrite(LED_PIN,HIGH);
    }
    else{
      digitalWrite(LED_PIN,LOW);
    }
    delay(2000); // Pause pour éviter un rafraîchissement trop rapide
  
}
