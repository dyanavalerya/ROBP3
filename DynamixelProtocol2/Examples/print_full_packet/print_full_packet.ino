#include <dynamixel2.h>

using namespace DynamixelProtocol2;

// Make an instance of the Dynamixel class (moror3 object)
Dynamixel motor3(3, 57600);


void setup() {
  // Serial port for transmitting data to the Dynamixel motors
  Serial3.begin(57600); 

  // Serial port for reading data transmitted to Mega by Dynamixel
  Serial.begin(57600); 

  motor3.SendPacket(Instructions::Read, Parameters::BaudRate, 0);
}

void loop() {
  // Set the RTS pin to reception mode
  RTS rts(5);
  rts.RTSReception();

  // Remember to change the size (11 + size of parameter)
  if (Serial3.available() >= 12) {
    motor3.ReceiveFullPacket(Instructions::Read, Parameters::BaudRate);
  }
}
