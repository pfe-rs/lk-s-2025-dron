#include <IMU_Fusion_SYC.h>
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

IMU imu(Wire);
#define RGB_BUILTIN 21
#define MAX_MOTOR 180

const double setpoint = 0;
const double K[3][3] = {
  //  P    I     D
  { 0,      0,    0 },    // pitch
  { 0,      0,    0 },    // roll
  { 0.1,    0,    0 }     // yaw
};

double input_pitch,   error_pitch,  errorSum_pitch,   lastError_pitch,  dError_pitch,   output_pitch;
double input_roll,    error_roll,   errorSum_roll,    lastError_roll,   dError_roll,    output_roll;
double input_yaw,     error_yaw,    errorSum_yaw,     lastError_yaw,    dError_yaw,     output_yaw;

const double Kp_pitch = K[0][0],  Ki_pitch = K[0][1],   Kd_pitch = K[0][2];
const double Kp_roll = K[1][0],   Ki_roll = K[1][1],    Kd_roll = K[1][2];
const double Kp_yaw = K[2][0],    Ki_yaw = K[2][1],     Kd_yaw = K[2][2];

double P_pitch = 0,   I_pitch = 0,  D_pitch = 0;
double P_roll = 0,    I_roll = 0,   D_roll = 0;
double P_yaw = 0,     I_yaw = 0,    D_yaw = 0;

unsigned long lastTime = 0;

Servo motori[4];
const int pinoviMotori[] = {7, 8, 9, 10};
int vrednosti[4];
int throttle;

void setup() {
  rgbLedWrite(RGB_BUILTIN, 15, 0, 0); // Informacija da smo poceli da radimo
  
  Serial.begin(115200);
  
  InicijalizacijaMotora();
  InicijalizacijaBluetootha();
  InicijalizacijaSenzora();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  KalibracijaMotora();
}

void loop() {
  unsigned long now = millis();

  PID(now, lastTime);
  
  ObradaDolaznihPodataka();
    
  PostaviVrednostiMotora();
  imu.Calculate();
  PlotPodataka();
  
  delay(1);
  
  // End of cycle
  lastTime = now;
}

void PlotPodataka() {
  Serial.print("Pitch: "); Serial.print(imu.getAngleY()); Serial.print(" ");
  Serial.print("Roll: "); Serial.print(-imu.getAngleX()); Serial.print(" ");
  Serial.print("Yaw: "); Serial.print(imu.getAngleZ()); Serial.println();
}

void ObradaDolaznihPodataka() {
  String rxValue = pRxCharacteristic->getValue();
  pRxCharacteristic->setValue("");

  if (rxValue.length() > 0) {
    throttle = map(rxValue.toInt(), 0, 100, 0, MAX_MOTOR);
    if (throttle == 0) for (int i = 0; i < 4; i++) vrednosti[i] = 0;
    else {
      vrednosti[0] = (throttle + output_pitch - output_roll - output_yaw);
      vrednosti[1] = (throttle - output_pitch - output_roll + output_yaw);
      vrednosti[2] = (throttle + output_pitch + output_roll + output_yaw);
      vrednosti[3] = (throttle - output_pitch + output_roll - output_yaw);
    
      for (int i = 0; i < 4; i++){
        if (vrednosti[i] < 0) vrednosti[i] = 0;
        else if (vrednosti[i] > MAX_MOTOR) vrednosti[i] = MAX_MOTOR;
      }
    }
  }
}

void PID(unsigned long now, unsigned long lastTime) {
  double dt = (double)(now - lastTime);
  imu.Calculate();

  // Pitch PID
  input_pitch = imu.getAngleY();
  
  error_pitch = setpoint - input_pitch;
  P_pitch = Kp_pitch * error_pitch;
  
  errorSum_pitch += (error_pitch * dt);
  I_pitch = Ki_pitch * errorSum_pitch;
  
  dError_pitch = (error_pitch - lastError_pitch) / dt;
  D_pitch = Kd_pitch * dError_pitch;

  output_pitch = P_pitch + I_pitch + D_pitch;
  
  lastError_pitch = error_pitch;

  // Roll PID
  input_roll = -imu.getAngleX();
  
  error_roll = setpoint - input_roll;
  P_roll = Kp_roll * error_roll;
  
  errorSum_roll += (error_roll * dt);
  I_roll = Ki_roll * errorSum_roll;
  
  dError_roll = (error_roll - lastError_roll) / dt;
  D_roll = Kd_roll * dError_roll;

  output_roll = P_roll + I_roll + D_roll;
  
  lastError_roll = error_roll;

  // Yaw PID
  input_yaw = imu.getAngleZ();
  
  error_yaw = setpoint - input_yaw;
  P_yaw = Kp_yaw * error_yaw;
  
  errorSum_yaw += (error_yaw * dt);
  I_yaw = Ki_yaw * errorSum_yaw;
  
  dError_yaw = (error_yaw - lastError_yaw) / dt;
  D_yaw = Kd_yaw * dError_yaw;

  output_yaw = P_yaw + I_yaw + D_yaw;
  
  lastError_yaw = error_yaw;
}

void InicijalizacijaMotora() {
  for (int i = 0; i < 4; i++){
    motori[i].setPeriodHertz(50);
    motori[i].attach(pinoviMotori[i], 1000, 2000);
  }
}

void InicijalizacijaBluetootha() {
  // Inicijalizacija Bluetooth-a
  BLEDevice::init("UART Service");
  pServer = BLEDevice::createServer();

  // Podesavanje servisa
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  // Paljenje servera i ucini ga vidljivim
  pService->start();
  pServer->getAdvertising()->start();
}

void InicijalizacijaSenzora() {
  // IIC protokol
  Wire.begin(13, 12);

  // Ziroskop i akcelometar
  imu.begin(CHOOSE_MPU6050);
  imu.MPU6050_CalcGyroOffsets();

  // Ultrazvucni senzor
  // treba da se doda...
}

void PostaviVrednostiMotora() {
  for (int i = 0; i < 4; i++){
    motori[i].write(vrednosti[i]);
  }
}

void KalibracijaMotora() {
  for (int i = 0; i < 4; i++) vrednosti[i] = MAX_MOTOR;
  PostaviVrednostiMotora();
  delay(500);

  for (int i = 0; i < 4; i++) vrednosti[i] = 0;
  PostaviVrednostiMotora();
  delay(500);
}
