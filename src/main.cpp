#include <Arduino.h>
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);
BLDCMotor motor = BLDCMotor(7, 10);


// put function declarations here:

void setup() {
  // Setup magnetic position sensor for motor
  Serial.begin(115200);
  Wire.setClock(400000);
  sensor.init();

  driver.voltage_power_supply = 12;

  if(!driver.init()) {
    Serial.println("Driver init failed!");
    return;
  }

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here: