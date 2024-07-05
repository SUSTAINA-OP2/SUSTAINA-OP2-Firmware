#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Firmware version
constexpr uint8_t FIRMWARE_VERSION = 0x00;  // Last updated: 2024/07/05

// Control-Switches-Board dependent settings
constexpr uint8_t BOARD_ID = 0xA0;

constexpr uint8_t CHIP_SELECT_PIN = 10; // ADS126XのCSピン

// Command definitions (borad original command: 0x00 ~ 0xCF)
constexpr uint8_t SEND_VAL_CMD = 0xA0;

// Error status definitions (borad original error status)
// constexpr uint8_t _ERROR = 0b00010000;
// constexpr uint8_t _ERROR = 0b00100000;
// constexpr uint8_t _ERROR = 0b01000000;
// constexpr uint8_t _ERROR = 0b10000000;

#endif // CONFIG_H
