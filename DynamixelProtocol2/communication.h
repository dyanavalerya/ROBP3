#ifndef DYNAMIXELPROTOCOL2_COMMUNICATION_H
#define DYNAMIXELPROTOCOL2_COMMUNICATION_H

#include "dynamixel2.h"

namespace DynamixelProtocol2 {

    /// The SendPacket function constructs a packet, given the input parameters set by the user. The packet
    /// is constructed according to the Dynamixel Protocol 2.0. Once all the fields are populated, it is
    /// being sent to the servo motor.
    /// \tparam T at compile time it figures out the data type of the inputData, based on its size (i.e. how big the number is)
    /// \param instruction Contains the address of the sent command, that the motor knows of, e.g. Read from the register
    /// \param parameter Contains the parameter that is being inputted to the command (e.g. present position)
    /// \param inputData This field is being used in case the user wants to Write something to the motor (e.g. a goal position)
    template<typename T>
    void Dynamixel::SendPacket(uint8_t instruction, uint8_t parameter = 0, T inputData = 0) {

        // Set RTS pin to transmission mode
        RTS rts(5);
        rts.RTSTransmission();

        uint8_t sendPacketLength = 0;

        // set the length of the packet based on the type of the instruction
        switch (instruction) {
            case Instructions::Ping: {
                sendPacketLength = 10; // length is always 10 bytes
                break;
            }
            case Instructions::Read: {
                sendPacketLength = 14; // length is always 14 bytes
                break;
            }
            case Instructions::Write: {
                sendPacketLength = 12 + parameterSizes[parameter];
                break;
            }
        }

        // Initialize an array that stores the packet data to be sent to the motor
        uint8_t sendPacket[sendPacketLength];

        // Populate the array

        // Header
        sendPacket[0] = 0xff;
        sendPacket[1] = 0xff;
        sendPacket[2] = 0xfd;
        sendPacket[3] = 0x00;
        // ID
        sendPacket[4] = this->_ID;
        // Length
        sendPacket[5] = (sendPacketLength - 7) & 0x00FF; // Length 1: LEN_L
        sendPacket[6] = ((sendPacketLength - 7) >> 8) & 0x00FF; // Length 2: LEN_H
        // Instruction
        sendPacket[7] = instruction;

        // Set the address of the chosen parameter, if the instruction is something else other than Ping
        if (instruction != Instructions::Ping) {
            sendPacket[8] = (parameterAddresses[parameter] & 0x00FF); // Param 1: Low-order byte from the starting address
            sendPacket[9] = (parameterAddresses[parameter] >> 8) & 0x00FF; // Param 2: High-order byte from the starting address
        }
        switch (instruction) {
            case Instructions::Write: {
                // Write inputData into a number of parameters based on the size, in bytes
                uint8_t inputDataArray[parameterSizes[parameter]];
                for (int j = 0; j < parameterSizes[parameter]; j++) {
                    inputDataArray[j] = (inputData >> (8 * j)) & 0x00FF;
                }

                // pass the values of the input data into the sendPacket array
                for (int i = 10; i < (10 + parameterSizes[parameter]); i++) {
                    sendPacket[i] = inputDataArray[i - 10];
                }
                break;
            }
            case Instructions::Read: {
                // Set the size of the parameter (e.g. present position = 4 bytes), that is going to be sent by the motor
                sendPacket[10] = (parameterSizes[parameter] & 0x00FF); // Param 3: Low-order byte from the data length
                sendPacket[11] = (parameterSizes[parameter] >> 8) & 0x00FF; // Param 4: High-order byte from the data length
                break;
            }
        }

        // Calculate the CRC for each distinct packet
        // The CRC is calculated from Header to Parameters, including both,
        // and stored in the CRC_L and CRC_H fields, which are the last fields of the sent and received packets
        uint16_t CRC_L, CRC_H;
        CalculateCRC_LAndCRC_H(sendPacket, &CRC_L, &CRC_H);

        // 16bit CRC field which checks if the Packet has been damaged during communication
        sendPacket[sendPacketLength - 2] = CRC_L; // CRC 1: CRC_L
        sendPacket[sendPacketLength - 1] = CRC_H; // CRC 2: CRC_H

        uint8_t len = sizeof(sendPacket) / sizeof(uint8_t);

        // Write the packet to the Dynamixel motor
        Serial3.write(sendPacket, len);

        // Wait for the previous instruction (write to the motors in this case)
        // to be completed before sending the next command
        Serial3.flush();
    }

    /// The ReceivePacket function reads the data from the serial port, that is being transmitted by the Dynamixel
    /// motor and reconstructs a part of the packet from it. That part is only consisted of the Parameter fields.
    /// \tparam parameterSize The size in bytes of the parameter from the Control Table
    /// (e.g. present position needs 4 bytes, baud rate needs 1 byte)
    /// \return an array with parameter data (e.g. P1 P2 P3 P4) arranged according to the Little-endian
    template<uint8_t parameterSize>
    struct ReturnArray<parameterSize> Dynamixel::ReceivePacket() {
        // create an array that contains the parameter data, that is sent by the motor
        ReturnArray<parameterSize> returnPacket;

        // initialize an array that will contain the data bytes until the length fields, including them
        // that will be 7 bytes = header(4) + id(1) + length(2)
        uint8_t lengthDataArray[7] = {0};

        // Set RTS pin to reception mode
        RTS rts(5);
        rts.RTSReception();

        unsigned int startTime = micros();

        // Make sure that there are 7 bytes of data coming through the serial port
        while (Serial3.available() < 7) {
            unsigned int endTime = micros();
            unsigned int time = endTime - startTime;
            // check for timeout exception
            // if the loop does not finish after 10 milliseconds, return out of the function
            if (time > 10000) {
                returnPacket.parameters[0] = 0;
                returnPacket.parameters[1] = 0;
                returnPacket.parameters[2] = 0;
                // return a packet of zeros (only when there is a timeout)
                return returnPacket;
            }
        }
        // We are interested in the first 7 bytes because that is from where we can extract information
        // about the total length of the return packet
        for (uint8_t i = 0; i < 7; i++) {
            lengthDataArray[i] = Serial3.read();
        }
        // the length fields are always found at positions 5 and 6
        // the length indicates the Byte size of Instruction, Parameters and CRC fields
        uint16_t length = ((lengthDataArray[6] << 8) + lengthDataArray[5]); // the one found in the received packet
        uint16_t totalPacketLength = 7 + length;
        // length - Instruction(1) - Error(1) - CRC(2)
        uint16_t numberOfParameters = length - 4;

        // Check for the rest of the packet that comes after the length bytes (LEN_H, LEN_L)
        while (Serial3.available() < length) {
            // do nothing
        }
        for (uint8_t j = 7; j < totalPacketLength; j++) {
            // Extract the parameters array from the status packet array (located after the error byte, i.e. starting at pos 9)
            if (j > 8 && j < (8 + numberOfParameters + 1)) {
                // i - 9, so the new array has its first value starting from position 0
                returnPacket.parameters[j - 9] = Serial3.read();
            } else {
                if (j == 8) {
                    uint8_t errorValue = Serial3.read();
                    // Check if there are any communication errors
                    InformationErrorHandling(errorValue);
                } else {
                    Serial3.read();
                }
            }
        }
        return returnPacket;
    }

    /// Reads/ gets data about the servo motor.
    /// \tparam parameter The parameter that is being read (e.g. present position, baud rate, etc.)
    /// \tparam T the type of the return function based on the size of the parameter
    /// (e.g. uint8_t for ID and uint32_t for present position)
    /// \return the value of the parameter from the Control Table (e.g. present position = 980, ID = 1)
    template<uint8_t parameter, typename T>
    T Dynamixel::Read() {
        // create the variable that is going to store the parameter value (e.g. present position)
        T currentParameterValue;

        Dynamixel motor(this->_ID, this->_baudRate);

        // Send the packet to the Dynamixel servo
        motor.SendPacket<uint8_t>(Instructions::Read, parameter, 0);

        // Collect data from the Serial port
        ReturnArray<parameterSizes[parameter]> returnPacket = motor.ReceivePacket<parameterSizes[parameter]>();

        uint8_t len = sizeof(returnPacket.parameters) / sizeof(uint8_t);

        // Convert data from hexadecimal to decimal
        for (uint8_t i = 0; i < len; i++) {
            // Every new value needs to be shifted 8 x i, because each byte has 8 bits
            // and with each iteration the number of bits increases by i
            // the part that the shift is anded with, makes sure that it clears the message
            // in case there are any bit flips
            currentParameterValue += (returnPacket.parameters[i] << (8 * i)) & ((0xFF) << (8 * i));
        }
        // Print e.g. 57600 instead of 1
        if (parameter == Parameters::BaudRate) {
            currentParameterValue = baudRatesArray[currentParameterValue];
        }
        //PrintMotorData(parameter, currentParameterValue);
        // return the parameter value
        // e.g. can be used for feedback control
        return currentParameterValue;
    }

    /// Writes/ Sets data to the servo motors.
    /// \tparam parameter the parameter that is being changed (e.g. velocity limit)
    /// \tparam T at compile time it figures out the data type of the inputData, based on its size (i.e. how big the number is)
    /// \param inputData the new value the parameter gets (e.g. change the velocity limit from 285 to 10 [rev/min])
    template<uint8_t parameter, typename T>
    void Dynamixel::Write(T inputData) {
        Dynamixel motor(this->_ID, this->_baudRate);

        // Send the packet to the Dynamixel servo
        motor.SendPacket(Instructions::Write, parameter, inputData);

        // Receive packet to check for errors if any
        motor.ReceivePacket<parameterSizes[parameter]>();
    }

}
#endif //DYNAMIXELPROTOCOL2_COMMUNICATION_H
