#include <Arduino.h>
#include <BLDCMotor.h>
#include <sensors/MagneticSensorI2C.h>
#include <drivers/BLDCDriver3PWM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);
BLDCMotor motor = BLDCMotor(7, 10);

MPU6050 mpu;
int const INTERRUPT_PIN = 2;
bool DMPReady = false;  
uint8_t MPUIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t FIFOBuffer[64]; 
Quaternion q;
VectorInt16 aa;
VectorInt16 gy;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float ypr[3];
float euler[3];

volatile bool mpuInterrupt = false;
void dmpDataReady();
void initializeMPU();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Wire.begin();
  Wire.setClock(400000);

  initializeMPU();

  sensor.init();

  driver.voltage_power_supply = 12;

  if(!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;

  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void initializeMPU() {
  // Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Serial.println(F("Testing MPU6050 connection..."));
  // if(mpu.testConnection() == false){
  //   // Serial.println("MPU6050 connection failed");
  //   while(true);
  // }
  // else {
  //   // Serial.println("MPU6050 connection successful");
  // }

  /*Wait for Serial input*/
  // Serial.println(F("\nSend any character to begin: "));
  // while (Serial.available() && Serial.read()); // Empty buffer
  // while (!Serial.available());                 // Wait for data
  // while (Serial.available() && Serial.read()); // Empty buffer again

  /* Initializate and configure the DMP*/
  // Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    // Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    // Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    // Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    // Serial.print(devStatus);
    // Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
}
