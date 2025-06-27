#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

void setup() {
  Serial.begin(115200);

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
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  if (Serial.available() > 0){
    String poruka = Serial.readStringUntil('\n');
    poruka += '\n';
    pTxCharacteristic->setValue(poruka);
    pTxCharacteristic->notify();
    delay(10);
  }
    
  String rxValue = pRxCharacteristic->getValue();
  pRxCharacteristic->setValue("");

  if (rxValue.length() > 0) {
    for (int i = 0; i < rxValue.length(); i++) {
      Serial.print(rxValue[i]);
    }

    Serial.println();
  }
  delay(100);
}
