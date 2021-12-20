#include <dynamixel2.h>
#include <communication.h>

using namespace DynamixelProtocol2;

#define numberOfMotors 5

// Create an array that contains the connected motors (i.e. the ones that respond)
Dynamixel motorList[numberOfMotors];

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
  }

  // Setup the motor configuration
  for (int i = 1; i < numberOfMotors + 1; i++) {
    // It is necessary to disable the torque for it to be possible to change 
    // the Control Table data of the EEPROM area
    motorList[i].Write<Parameters::TorqueEnable>(0);
    // Set the new settings
    motorList[i].Write<Parameters::VelocityLimit>(10);
    motorList[i].Write<Parameters::CurrentLimit>(200);
    motorList[i].Write<Parameters::AccelerationLimit>(6000);
  }
}

void loop() {


}
