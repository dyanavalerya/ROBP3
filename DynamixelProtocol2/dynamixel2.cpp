#include "dynamixel2.h"
#include "communication.h"

using namespace DynamixelProtocol2;

/// Finds the position of the input parameter "seek" from the input array.
/// \param arr an array of unsigned integers
/// \param len the length of the array
/// \param seek a value from the input array
/// \return the position of "seek" in the input array
uint8_t Dynamixel::findPos(uint32_t arr[], uint8_t len, uint32_t seek)
{
    for (uint8_t i = 0; i < len; ++i)
    {
        if (arr[i] == seek) return i;
    }
    return -1;
}

/// Empty constructor of the motor class - Dynamixel
Dynamixel::Dynamixel(){
}

/// Constructor for the motor class - Dynamixel
/// \param ID ID of the motor
/// \param baudRate baud rate of the motor
Dynamixel::Dynamixel(uint8_t ID, uint32_t baudRate){
    _ID = ID;
    _baudRate = findPos(baudRatesArray, 8, baudRate);
}

/// Calculates the CRC, given the packet data and the size of the data
/// \param crc_accum the variable in which the initial CRC value gets stored
/// \param data_blk_ptr the pointer to the address of the first value in the packet
/// \param data_blk_size the size of the packet excluding the CRC fields
/// \return the CRC value
uint16_t Dynamixel::update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size){
    uint8_t i, j;
    uint16_t crc_table[256] = {
            0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
            0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
            0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
            0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
            0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
            0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
            0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
            0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
            0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
            0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
            0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
            0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
            0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
            0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
            0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
            0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
            0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
            0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
            0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
            0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
            0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
            0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
            0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
            0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
            0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
            0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
            0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

/// Splits the CRC value according to Little-endian, into least and most significant bytes
/// \param sendPacket the pointer to the address of the first value in the packet
/// \param CRC_L the least significant byte of the CRC
/// \param CRC_H the most significant byte of the CRC
void Dynamixel::CalculateCRC_LAndCRC_H(uint8_t *sendPacket, uint16_t *CRC_L, uint16_t *CRC_H) {

    // Instruction and Parameters memory allocation in bytes, sendPacket[6] = LEN_H, sendPacket[5] = LEN_L
    // that are always at that location in the Packet, according to Dynamixel Protocol 2.0
    uint16_t packet_length = ((sendPacket[6] << 8 ) + sendPacket[5]);

    // Number of bytes in the Packet excluding the CRC
    // without CRC (Header(3) + Reserved(1) + Packet ID(1) + Length(2) + Packet Length - CRC(2))
    uint16_t data_blk_size = 5 + packet_length;

    // Find full CRC for the transmitted data, given the protocol divider = 0x8005
    uint16_t CRC = update_crc(0, sendPacket , data_blk_size);

    //Little-endian
    *CRC_L = (CRC & 0x00FF);
    *CRC_H = (CRC >> 8) & 0x00FF;
}

/// RS485 breadboard Constructor
/// \param the digital pin on the Arduino that is connected to the RTS pin on the RS485 breadboard
RTS::RTS(int RTSpin)
{
    pinMode(RTSpin, OUTPUT);
    _RTSpin = RTSpin;
}

/// Set RTS pin of the RS485 Breakout board to transmission mode
void RTS::RTSTransmission(){
    digitalWrite(_RTSpin, HIGH);
}

/// Set RTS pin of the RS485 Breakout board to receiving mode
void RTS::RTSReception(){
    digitalWrite(_RTSpin, LOW);
}

void Dynamixel::PrintCurrentID(){
    Serial.print("Motor ");
    Serial.print(this->_ID);
    Serial.print(" has ");
}

/// Prints data about the Dynamixel motor (e.g. baud rate)
/// \param parameter the parameter that the user wants to know about the motor
/// \param currentParameterValue the parameter value
void Dynamixel::PrintMotorData(uint8_t parameter, uint32_t currentParameterValue){
    // Set the first letter in string to uppercase
    parameterNames[parameter][0] = toupper(parameterNames[parameter][0]);

    Serial.print(parameterNames[parameter]);
    Serial.print(" for motor "); // TODO: need to use enum Parameters name in it, depending on the input
    Serial.print(this->_ID);
    Serial.print(" is: ");
    Serial.println(currentParameterValue);
}

/// Calculates the number of parameters from any received packet.
/// \return number of parameters
const uint8_t Dynamixel::GetNumberOfParameters() {
    uint8_t lengthDataArray[7] = {0};

    // Set RTS pin to reception mode
    RTS rts(5);
    rts.RTSReception();

    while (Serial3.available() < 7){
        // do nothing
    }

    for (int i = 0; i < 7; i++) {
        lengthDataArray[i] = Serial3.read();
    }
    // the length is calculated with the data received from the motor
    uint16_t length = ((lengthDataArray[6] << 8 ) + lengthDataArray[5]);
    uint16_t numberOfParameters = length - 4;

    return numberOfParameters;
}

/// Prints the entire received packet in hexadecimal.
/// \param instruction the instruction address
/// \param parameter the parameter type
void Dynamixel::ReceiveFullPacket(uint8_t instruction, uint8_t parameter) {
    uint8_t length = 0;
    switch (instruction) {
        case Instructions::Write:{
            length = 11;
            break;
        }
        case Instructions::Read:{
            length = 11 + parameterSizes[parameter];
            break;
        }
    }
    Serial.println("Status packet: ");
    for (int i = 0; i < length; i++) {
        Serial.println(Serial3.read(), HEX);
    }
}

/// Checks if a motor exists and gets basic information about it.
/// \param modelNumber a pointer to the model number value
/// \param firmwareVersion a pointer to the firmware version value
void  Dynamixel::Ping(uint16_t *modelNumber, uint8_t *firmwareVersion) {
    *modelNumber = 0;
    *firmwareVersion = 0;
     Dynamixel motor(this->_ID, this->_baudRate);

     // Send the packet to the Dynamixel servo
     motor.SendPacket<uint8_t>(Instructions::Ping, 0, 0);

     // Collect 3 bytes from the Serial port
     // This is known from the Protocol 2.0 that the Ping instruction returns 3 bytes of data
     ReturnArray<3> returnPacket = motor.ReceivePacket<3>();

     uint8_t len = sizeof(returnPacket.parameters) / sizeof(uint8_t);
//        Serial.print("return packet len: ");
//        Serial.println(len);
     for (uint8_t i = 0; i < len; i++)
     {
         if (i > -1 && i < 2){
             *modelNumber += (returnPacket.parameters[i] << (8 * i)) & ((0xFF) << (8 * i));
         }
         else{
             *firmwareVersion = returnPacket.parameters[i];
         }
     }
 }

 /// Checks for any communication errors that can happen when sending and receiving packets.
 /// \param errorValue the value from the error field in the status packet
 void Dynamixel::InformationErrorHandling(uint8_t errorValue){
     switch (errorValue) {
         case 0x01:{
             PrintCurrentID();
             // Failed to process the sent Instruction Packet
             Serial.println("Result fail");
             break;
         }
         case 0x02:{
             PrintCurrentID();
             // Undefined Instruction has been used
             // Action has been used without Reg Write
             Serial.println("Instruction Error");
             break;
         }
         case 0x03:{
             PrintCurrentID();
             // CRC of the sent Packet does not match
             Serial.println("CRC Error");
             break;
         }
         case 0x04:{
             PrintCurrentID();
             // Data to be written in the corresponding Address is outside the range of the minimum/maximum value
             Serial.println("Data Range Error");
             break;
         }
         case 0x05:{
             PrintCurrentID();
             // Attempt to write Data that is shorter than the data length of the corresponding Address
             // (ex: when you attempt to only use 2 bytes of an item that has been defined as 4 bytes)
             Serial.println("Data Length Error");
             break;
         }
         case 0x06:{
             PrintCurrentID();
             // Data to be written in the corresponding Address is outside the Limit value
             Serial.println("Data Limit Error");
             break;
         }
         case 0x07:{
             PrintCurrentID();
             // Attempt to write a value in an Address that is Read Only or has not been defined
             // Attempt to read a value in an Address that is Write Only or has not been defined
             // Attempt to write a value in the ROM domain while in a state of Torque Enable(ROM Lock)
             Serial.println("Access Error");
             break;
         }
     }
}

