#include <dynamixel2.h>
#include <communication.h>

using namespace DynamixelProtocol2;

#define numberOfMotors 5

// Create an array that contains the connected motors (i.e. the ones that respond)
Dynamixel motorList[numberOfMotors];

int positions[5] = {1600, 2254, 743, 2054, 2022};
//int positions[5] = {1900, 2000, 1600, 2154, 1900};

void setup() {
  // Serial port for transmitting data to the Dynamixel motors
  Serial3.begin(57600);

  // Serial port for reading data transmitted to Mega by Dynamixel
  Serial.begin(57600);

  // Initialize the parameters that get received as a result of the Ping instruction
  uint16_t modelNumber = 0;
  uint8_t firmVersion = 0;

  for (uint8_t i = 1; i < numberOfMotors + 1; i++) {
    // create Dynamixel motor objects
    Dynamixel motor(i, 57600);

    // Check the existence of the motor devices and basic information
    motor.Ping(&modelNumber, &firmVersion);

    // If the motors do not respond, then the values of the model number and firmware version
    // will remain the same as their initial values (zero and zero)
    // in that case go to the next iteration and skip the next steps
    if (modelNumber == 0 && firmVersion == 0) {
      continue;
    }

    // add the motor that responded into the motorList array
    motorList[i] = Dynamixel(i, 57600);

    // Print data about each motor
    Serial.print("Model number and firmware version for motor ");
    Serial.print(motor._ID);
    Serial.print(" are: ");
    Serial.print(modelNumber);
    Serial.print(" and ");
    Serial.println(firmVersion);
  }

  // Set commands to the motors
  for (int i = 1; i < numberOfMotors + 1; i++) {
    motorList[i].Write<Parameters::TorqueEnable>(1);
    motorList[i].Write<Parameters::GoalVelocity>(10);
    motorList[i].Write<Parameters::GoalCurrent>(200);
    motorList[i].Write<Parameters::GoalPosition>(positions[i - 1]);
  }
}

void loop() {

}
