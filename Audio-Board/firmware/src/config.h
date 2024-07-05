#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Firmware version
constexpr uint8_t FIRMWARE_VERSION = 0x01;  // Last updated: 2024/07/05

// Audio-Board dependent settings
constexpr uint8_t BOARD_ID = 0xA3;

// DFPlayer settings
constexpr uint32_t PLAYER_SERIAL_BAUDRATE = 9600;
constexpr uint8_t PLAYER_RX_PIN = 4;
constexpr uint8_t PLAYER_TX_PIN = 3;
#define PLAYER_SERIAL Serial2

// Command definitions (borad original command: 0x00 ~ 0xCF)
constexpr uint8_t PLAY_SOUND_CMD = 0xA0;
constexpr uint8_t PLAYER_RESET_CMD = 0xB0;

// Error status definitions (borad original error status)
// constexpr uint8_t _ERROR = 0b00010000;
// constexpr uint8_t _ERROR = 0b00100000;
// constexpr uint8_t _ERROR = 0b01000000;
// constexpr uint8_t _ERROR = 0b10000000;

#endif // CONFIG_H
