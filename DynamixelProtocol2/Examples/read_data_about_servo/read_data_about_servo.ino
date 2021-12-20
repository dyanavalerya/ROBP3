#include <dynamixel2.h>
#include <communication.h>

using namespace DynamixelProtocol2;

#define numberOfMotors 5

// Make instances of the Dynamixel class
Dynamixel motor1(1, 57600);
Dynamixel motor2(2, 57600);
Dynamixel motor3(3, 57600);
Dynamixel motor4(4, 57600);
Dynamixel motor5(5, 57600);

// Save the Dynamixel class objects into an array of type Dynamixel
Dynamixel motorList[numberOfMotors] = {motor1, motor2, motor3, motor4, motor5};

void setup() {
  // Serial port for transmitting data to the Dynamixel motors
  Serial3.begin(57600);

  // Serial port for reading data transmitted to Mega by Dynamixel
  Serial.begin(57600);

  // Method 1: include the print function in the Read function
  for (int i = 0; i < numberOfMotors; i++) {
    motorList[i].Read<Parameters::CurrentLimit,  uint16_t>();
  }
  // Method 2: use the print function directly here and comment the one in the Read function
  //  uint32_t currentParameterValue3 = motor3.Read<Parameters::PresentPosition,  uint32_t>();
  //  motor3.PrintMotorData(Parameters::PresentPosition, currentParameterValue3);
}

void loop() {

}
