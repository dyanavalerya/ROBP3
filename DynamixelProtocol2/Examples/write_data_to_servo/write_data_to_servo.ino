#include <dynamixel2.h>
#include <communication.h>

using namespace DynamixelProtocol2;

#define numberOfMotors 5

Dynamixel motor1(1, 57600);
Dynamixel motor2(2, 57600);
Dynamixel motor3(3, 57600);
Dynamixel motor4(4, 57600);
Dynamixel motor5(5, 57600);

Dynamixel motorList[numberOfMotors] = {motor1, motor2, motor3, motor4, motor5};
int positions[numberOfMotors] = {1600, 2254, 743, 2054, 2022};
//int positions[numberOfMotors] = {1600, 2254, 1000, 2054, 2022};
//int positions[5] = {1900, 2000, 1100, 2154, 1900};

void setup() {
  // Serial port for transmitting data to the Dynamixel motors
  Serial3.begin(57600);

  // Serial port for reading data transmitted to Mega by Dynamixel
  Serial.begin(57600);

  // Example using GoalPosition with multiple motors (Position Control Mode)
  for (int i = 0; i < numberOfMotors; i++) {
    // Torque needs to be enabled in order to be able to move the motor
    motorList[i].Write<Parameters::TorqueEnable>(1);
    motorList[i].Write<Parameters::GoalPosition>(positions[i]);
  }
  // Example using GoalPWM with one motor (PWM Control Mode)

  // Disable the torque to be able to write to the EEPROM area
  // motor3.Write<Parameters::TorqueEnable>(0);

  // Change the operating mode only once, then comment it for the next times
  // since it it only neccessary to change it once
  //  motor3.Write<Parameters::OperatingMode>(16);

  //  motor3.Write<Parameters::GoalPWM>(200);
  //  delay(2000);
  //  motor3.Write<Parameters::GoalPWM>(0);
}

void loop() {

  //  motor3.Write<Parameters::GoalPosition>(900);
  //  delay(180);
  //  motor3.Write<Parameters::GoalPosition>(1200);
  //  delay(200);
}
