#include <dynamixel2.h>
#include <communication.h>

using namespace DynamixelProtocol2;

// Make an instance of the Dynamixel class
Dynamixel motor1(1, 57600);


void setup() {
  // Serial port for transmitting data to the Dynamixel motors
  Serial3.begin(57600); 
  
  // Serial port for reading data transmitted to Mega by Dynamixel
  Serial.begin(57600); 

  motor1.SendPacket<uint8_t>(Instructions::Read, Parameters::ServoID, 0);
}

void loop() {
  // Set the RTS pin of the RS485 breadboard to reception mode
  RTS rts(5);
  rts.RTSReception();

  // Make sure to manually change the amount of bytes the motor is sending (11 + nr of parameters)
  // In this case ServoID info is contained in only 1 byte)
  if (Serial3.available() >= 12) {
    // struct ReturnArray<nrOfParameters> returnPacket = motor1.ReceivePacket<nrOfParameters>();
    struct ReturnArray<1> returnPacket = motor1.ReceivePacket<1>();

    uint8_t len = sizeof(returnPacket.parameters) / sizeof(uint8_t);

    //Serial.println(len);
    for (uint8_t i = 0; i < len; i++) {
      Serial.println(returnPacket.parameters[i], HEX);
    }
  }
}
