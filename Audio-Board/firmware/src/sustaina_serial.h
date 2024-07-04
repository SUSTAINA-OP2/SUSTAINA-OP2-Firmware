#ifndef SUSTAINA_SERIAL_H
#define SUSTAINA_SERIAL_H

#include <Arduino.h>
#include <vector>

using namespace std;

// RS485 settings
constexpr uint8_t RS485_RX_PIN = 0;
constexpr uint8_t RS485_TX_PIN = 1;
constexpr uint8_t RS485_TXDEN_PIN = 2;
constexpr uint32_t RS485_SERIAL_BAUDRATE = 1000000;
#define RS485_SERIAL Serial1

// Packet settings
constexpr uint8_t HEADER[] = { 0xFE, 0xFE };
constexpr size_t HEADER_LENGTH = sizeof(HEADER);
constexpr size_t CRC_LENGTH = sizeof(uint16_t);
constexpr size_t ADDRESS_LENGTH = sizeof(uint8_t);

// RX packet lengths
// RX packet: headder + (id + length + command + option ) + rxData + crc
constexpr size_t RX_PACKET_FORWARD_LENGTH = HEADER_LENGTH + 4;
constexpr size_t RX_PACKET_MIN_LENGTH = RX_PACKET_FORWARD_LENGTH + CRC_LENGTH;

// TX packet lengths
// TX packet: headder + (id + length + command(mask)  + error) + txData + crc
constexpr size_t TX_PACKET_MIN_LENGTH = HEADER_LENGTH + 4 + CRC_LENGTH;

// Command definitions (common command 0xD0 ~ 0xFD)
constexpr uint8_t CHECK_FIRMWARE_CMD = 0xD0;
constexpr uint8_t CPU_RESET_CMD = 0xE0; // no reply.

// Error status definitions (common error status)
constexpr uint8_t CRC_ERROR = 0b00000001;
constexpr uint8_t CMD_PROCESS_ERROR = 0b00000010;
constexpr uint8_t NOT_EXISTENT_CMD_ERROR = 0b00000100;
// constexpr uint8_t _ERROR = 0b00001000;

class SUSTAINA_PMX_SERIAL {
private: 
    vector<uint8_t> rxPacket;
    static constexpr uint8_t RETURN_CMD_MASK = 0b01111111;
    bool checkHeader();

public:
    uint8_t rxId;
    size_t rxLength;
    uint8_t rxCommand;
    uint8_t rxOption;
    uint8_t txError = 0b00000000;

    vector<uint8_t> txData;

    void setupSerial();
    bool readPacket();
    bool checkCRCandID();
    void sendPacket();
    bool commandProcess() ;
};
#endif
