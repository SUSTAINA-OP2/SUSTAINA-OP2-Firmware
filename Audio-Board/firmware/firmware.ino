/** 
* @file firmware.ino
* @brief  Firmware for SUSTAINA-Audio-Boardd to be installed in SUSTAINA-OP2™
* @author Masato Kubotera
* @date 2024/07/05
* @version 0x01
* @copyright SUSTAINA-OP2™ 2024
*/

#include "Arduino.h"
#include "src/DFRobotDFPlayerMini/DFRobotDFPlayerMini.h"
#include <vector>
#include "src/config.h"
#include "src/sustaina_serial.h"

// #define SERIAL_DEBUG

// USB Serial
constexpr uint32_t USB_SERIAL_BAUDRATE = 115200;

DFRobotDFPlayerMini Player;
SUSTAINA_PMX_SERIAL SutainaSerial;

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(USB_SERIAL_BAUDRATE);
  while (!Serial) {
    ;  // Wait for USB serial port to connect. Needed for native USB
  }
#endif  // SERIAL_DEBUG

  // RS485 Serial setup
  SutainaSerial.setupSerial();

  // DF Player Serial setup
  PLAYER_SERIAL.begin(PLAYER_SERIAL_BAUDRATE, SERIAL_8N1, PLAYER_RX_PIN, PLAYER_TX_PIN);
  while (!Player.begin(PLAYER_SERIAL, /*isACK = */ true, /*doReset = */ true)) {
    ;  // wait until DFPlayer is ready
  }

  Player.volume(10);  // Set volume value. From 0 to 30
  Player.play(1);     // Play the first mp3
}

void loop() {
  if (SutainaSerial.readPacket()) {
    if (SutainaSerial.checkCRCandID() && !SutainaSerial.commandProcess()) {
      switch (SutainaSerial.rxCommand) {
        case PLAY_SOUND_CMD:
          if (SutainaSerial.rxOption < 100) {
            Player.play(static_cast<int16_t>(SutainaSerial.rxOption));
            break;
          }
          SutainaSerial.txError |= SutainaSerial.CMD_PROCESS_ERROR;
          break;

        case PLAYER_RESET_CMD:
          Player.reset();
          break;

        default:
          SutainaSerial.txError |= SutainaSerial.NOT_EXISTENT_CMD_ERROR;
      }
    }

    SutainaSerial.sendPacket();
  }
}
