#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Firmware version
constexpr uint8_t FIRMWARE_VERSION = 0x01;  // Last updated: 2024/07/05

// Control-Switches-Board dependent settings
constexpr uint8_t BOARD_ID = 0xA1;

constexpr uint8_t NEOPIXEL_LED_PIN = 6; // GPIO pin number of MICRO Arduino PIN: D3

constexpr uint8_t G_SWITCH_PIN = 4;
constexpr uint8_t R_SWITCH_PIN = 5;

constexpr uint8_t R_LED_PIN = 14;
constexpr uint8_t G_LED_PIN = 15;
constexpr uint8_t B_LED_PIN = 16;

constexpr uint8_t NUM_NEOPIXEL = 4;

// Command definitions (borad original command: 0x00 ~ 0xCF)
constexpr uint8_t GET_STATE_AND_SET_LED_CMD = 0xA0;

// Error status definitions (borad original error status)
// constexpr uint8_t _ERROR = 0b00010000;
// constexpr uint8_t _ERROR = 0b00100000;
// constexpr uint8_t _ERROR = 0b01000000;
// constexpr uint8_t _ERROR = 0b10000000;

constexpr uint16_t CONSECUTIVE_STRIKE_JUDGMENT_SECONDS = 1500; //ms
constexpr uint16_t STRIKE_COUNT = 3;
constexpr uint16_t CHATTERING_PREVENTION_SECONDS = 100; //ms
#endif // CONFIG_H
