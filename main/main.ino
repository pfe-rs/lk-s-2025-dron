#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define RGB_BUILTIN 21

Servo motori[4];
const int pinoviMotori[] = {7, 8, 9, 10};
int vrednosti[4];

void setup() {
  Serial.begin(115200);

  InicijalizacijaMotora();
  rgbLedWrite(RGB_BUILTIN, 15, 0, 0);  
  
  BLEDevice::init("UART Service");
  pServer = BLEDevice::createServer();

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
}

void loop() {
//  if (Serial.available() > 0){
//    String poruka = Serial.readStringUntil('\n');
//    poruka += '\n';
//    pTxCharacteristic->setValue(poruka);
//    pTxCharacteristic->notify();
//    delay(10);
//  }
    
  String rxValue = pRxCharacteristic->getValue();
  pRxCharacteristic->setValue("");

  if (rxValue.length() > 0) {
    if (rxValue.length() <= 3) for (int i = 0; i < 4; i++) vrednosti[i] = rxValue.toInt();
    else {
      int prosli_zarez = -1;
  
      for (int i = 0; i < 3; i++){
        int zarez = rxValue.indexOf(',', prosli_zarez + 1);
        vrednosti[i] = map(rxValue.substring(prosli_zarez + 1, zarez).toInt(), 0, 100, 0, 180);
        prosli_zarez = zarez;
      }
      vrednosti[3] = map(rxValue.substring(prosli_zarez + 1).toInt(), 0, 100, 0, 180);
      
      PostaviVrednostiMotora();
  
      for (int i = 0; i < 4; i++) {
        Serial.print(vrednosti[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  delay(100);
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
