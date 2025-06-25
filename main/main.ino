#include <ESP32Servo.h>

#define RGB_BUILTIN 21

Servo motori[4];
const int pinoviMotori[] = {7, 8, 9, 10};
int vrednosti[4];
String input;

void setup() {
  rgbLedWrite(RGB_BUILTIN, 10, 0, 0);
  Serial.begin(115200);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  InicijalizacijaMotora();
}

void loop() {
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');

    int prosli_zarez = -1;

    for (int i = 0; i < 3; i++){
      int zarez = input.indexOf(',', prosli_zarez + 1);
      vrednosti[i] = map(input.substring(prosli_zarez + 1, zarez).toInt(), 0, 100, 0, 180);
      prosli_zarez = zarez;
    }
    vrednosti[3] = map(input.substring(prosli_zarez + 1).toInt(), 0, 100, 0, 180);
  }
  
  PostaviVrednostiMotora();
}

void InicijalizacijaMotora(){
  for (int i = 0; i < 4; i++){
    motori[i].setPeriodHertz(50);
    motori[i].attach(pinoviMotori[i], 1000, 2000);
  }
}

void PostaviVrednostiMotora() {
  for (int i = 0; i < 4; i++){
    motori[i].write(vrednosti[i]);
  }
}
