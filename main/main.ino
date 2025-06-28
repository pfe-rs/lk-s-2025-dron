#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define RGB_BUILTIN 21

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

Servo motori[4];
const int pinoviMotori[] = {7, 8, 9, 10};
int vrednosti[4];
String input;

void setup() {
  rgbLedWrite(RGB_BUILTIN, 15, 0, 0);
  Serial.begin(115200);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  InicijalizacijaMotora();
  
  BLEDevice::init("НАЈЈАЧИ ДРОН");
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
}

void loop() {
  String input = pRxCharacteristic->getValue();
  pRxCharacteristic->setValue("");
  Serial.println(input);

  int prosli_zarez = -1;

  for (int i = 0; i < 3; i++){
    int zarez = input.indexOf(',', prosli_zarez + 1);
    vrednosti[i] = map(input.substring(prosli_zarez + 1, zarez).toInt(), 0, 100, 0, 180);
    prosli_zarez = zarez;
  }
  vrednosti[3] = map(input.substring(prosli_zarez + 1).toInt(), 0, 100, 0, 180);
  
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
