#ifndef DYNAMIXELPROTOCOL2_DYNAMIXEL2_H
#define DYNAMIXELPROTOCOL2_DYNAMIXEL2_H

#include "Arduino.h"

namespace DynamixelProtocol2{

    enum Instructions{
        Ping = 0x01,
        Read = 0x02,
        Write = 0x03,
        RegRead = 0x82,
        RegWrite = 0x83
    };
    static constexpr uint8_t parameterAddresses[64] = {0, 2, 6, 7, 8, 9, 10, 11, 12, 13, 20, 24, 31, 32, 34, 36, 38, 40,
                                                       44, 48, 52, 63, 64, 65, 68, 69, 70, 76, 78, 80, 82, 84, 88, 90,
                                                       98, 100, 102, 104, 108, 112, 116, 120, 122, 123, 124, 126, 128,
                                                       132, 136, 140, 144, 146, 168, 170, 172, 218, 220, 222, 224, 225,
                                                       226, 249, 250, 251};
    static constexpr uint8_t parameterSizes[64] = {2, 4, 1, 1, 1, 1, 1, 1, 1, 1, 4, 4, 1, 2, 2, 2, 2, 4, 4, 4, 4, 1, 1,
                                                   1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 4, 4, 4, 4, 2, 1, 1, 2, 2,
                                                   4, 4, 4, 4, 2, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1}; // in bytes

   static String parameterNames[64] = {"model number", "model information", "firmware version", "ID", "baud rate",
                                     "return delay time", "drive mode", "operating mode", "secondary ID",
                                     "protocol type", "homing offset", "moving threshold", "temperature limit",
                                     "max voltage limit", "min voltage limit", "PWM limit", "current limit",
                                     "acceleration limit", "velocity limit", "max position limit", "min position limit",
                                     "shutdown", "torque enable", "LED", "status return level", "registered instruction",
                                     "hardware error status", "velocity I gain", "velocity P gain", "position D gain",
                                     "position I Gain", "position P gain", "feedforward 2nd gain", "feedforward 1st gain",
                                     "BUS Watchdog", "goal PWM", "goal current", "goal velocity", "profile acceleration",
                                     "profile velocity", "goal position", "realtime tick", "moving", "movingStatus",
                                     "present PWM", "present current", "present velocity", "present position",
                                     "velocity trajectory", "position trajectory", "present input voltage",
                                     "present temperature", "indirectAddress1", "indirectAddress2", "indirectAddress3",
                                     "indirectAddress26", "indirectAddress27", "indirectAddress28", "indirectData1",
                                     "indirectData2", "indirectData3", "indirectData26", "indirectData27", "indirectData28" };

    // Parameters for the Read and Write instructions (Control Table)
    enum Parameters : uint8_t{
        ModelNumber,       // R 0
        ModelInformation,  // R 2
        FirmwareVersion,   // R 6
        ServoID,           // RW 7
        BaudRate,          // RW 8
        ReturnDelayTime,   // RW 9
        DriveMode,         // RW 10
        OperatingMode,     // RW 11
        SecondaryID,       // RW 12
        ProtocolType,      // RW 13
        HomingOffset,      // RW 20
        MovingThreshold,   // RW 24
        TemperatureLimit,  // RW 31
        MaxVoltageLimit,   // RW 32
        MinVoltageLimit,   // RW 34
        PWMLimit,          // RW 36
        CurrentLimit,      // RW 38
        AccelerationLimit, // RW 40
        VelocityLimit,     // RW 44
        MaxPosLimit,       // RW 48
        MinPosLimit,       // RW 52
        Shutdown,          // RW 63
        TorqueEnable,      // RW 64
        LED,               // RW 65
        StatusReturnLevel, // RW 68
        RegisteredInstruction,// R 69
        HardwareErrStatus, // R 70
        VelocityIGain,     // RW 76
        VelocityPGain,     // RW 78
        PositionDGain,     // RW 80
        PositionIGain,     // RW 82
        PositionPGain,     // RW 84
        Feedforward2ndGain,// RW 88
        Feedforward1stGain,// RW 90
        BUSWatchdog,       // RW 98
        GoalPWM,           // RW 100
        GoalCurrent,       // RW 102
        GoalVelocity,      // RW 104
        ProfileAcceleration,// RW 108
        ProfileVelocity,   // RW 112
        GoalPosition,      // RW 116
        RealtimeTick,      // R 120
        Moving,            // R 122
        MovingStatus,      // R 123
        PresentPWM,        // R 124
        PresentCurrent,    // R 126
        PresentVelocity,   // R 128
        PresentPosition,   // R 132
        VelocityTrajectory,// R 136
        PositionTrajectory,// R 140
        PresentInputVoltage,// R 144
        PresentTemperature,// R 146
        IndirectAddress1,  // RW 168
        IndirectAddress2,  // RW 170
        IndirectAddress3,  // RW 172
        IndirectAddress26, // RW 218
        IndirectAddress27, // RW 220
        IndirectAddress28, // RW 222
        IndirectData1,     // RW 224
        IndirectData2,     // RW 225
        IndirectData3,     // RW 226
        IndirectData26,    // RW 249
        IndirectData27,    // RW 250
        IndirectData28     // RW 251
    };

    static constexpr uint32_t baudRatesArray[8] = {9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000};
    enum BaudRates{
        Baud9600 = 0,
        Baud57600 = 1, // Default
        Baud115200 = 2,
        Baud1M = 3,
        Baud2M = 4,
        Baud3M = 5,
        Baud4M = 6,
        Baud4M5 = 7
    };

    template< uint8_t parameterSize >
    struct ReturnArray {
        uint8_t parameters[parameterSize];
    };

    /// The Dynamixel servo motor class, which contains functions that can be used to interact with the motors.
    class Dynamixel {
    public:
        uint8_t findPos(uint32_t arr[], uint8_t len, uint32_t seek);
        // Constructor
        Dynamixel();
        Dynamixel(uint8_t ID, uint32_t baudRate);
        uint8_t _ID;
        uint8_t _baudRate;

        // Functions
        uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
        void CalculateCRC_LAndCRC_H(uint8_t *sendPacket, uint16_t *CRC_L, uint16_t *CRC_H);

        template<typename T>
        void SendPacket(uint8_t instruction, uint8_t parameter, T inputData);

        template< uint8_t parameterSize >
        struct ReturnArray< parameterSize > ReceivePacket();
        void InformationErrorHandling(uint8_t errorValue);

        void ReceiveFullPacket(uint8_t instruction, uint8_t parameter);
        void PrintCurrentID();
        void PrintMotorData(uint8_t parameter, uint32_t currentParameterValue);
        const uint8_t GetNumberOfParameters();

        void Ping(uint16_t *modelNumber, uint8_t *firmwareVersion);
        template< uint8_t parameter, typename T >
        T Read();
        template< uint8_t parameter, typename T>
        void Write(T inputData);
    };

    /// The RTS class takes care of the RTS pin on the RS485 Breakout board
    class RTS
    {
    public:
        RTS(int RTSpin);
        void RTSTransmission();
        void RTSReception();
    private:
        int _RTSpin;
    };
}

#endif //DYNAMIXELPROTOCOL2_DYNAMIXEL2_H
