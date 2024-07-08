/** 
* @file firmware.ino
* @brief  Firmware for SUSTAINA-StrainGauge-Sensing-Board to be installed in SUSTAINA-OP2™
* @author Riku Yokoo, Masato Kubotera
* @date 2024/07/07
* @version 0x01
* @copyright SUSTAINA-OP2™ 2024
* @copyright Hamburg Bit-Bots 2019
*/

#include "Arduino.h"
#include "src/config.h"
#include "src/sustaina_serial.h"
#include "./src/ADS126X/ADS126X.h"
#include <array>
#include <vector>

// #define SERIAL_DEBUG

// USB Serial
constexpr uint32_t USB_SERIAL_BAUDRATE = 115200;

SUSTAINA_PMX_SERIAL SutainaSerial;

#define NUM_AVG 5  //number of samples to use for comparing the value to the median

ADS126X adc;

std::array<int32_t, 4> force;
std::array<uint8_t, 4> pos_pin{ 0, 2, 4, 6 };
std::array<uint8_t, 4> neg_pin{ 1, 3, 5, 7 };

size_t current_sensor = 0;
long _count = 0;

std::array<std::array<int32_t, NUM_AVG>, 4> previous_values;

void initADS() {
  //reset ads1262
  digitalWrite(CHIP_SELECT_PIN, LOW);
  delayMicroseconds(2);  // minimum of 4 t_clk = 0.54 us
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  delayMicroseconds(4);  // minimum of 8 t_clk = 1.09 us

  adc.begin(CHIP_SELECT_PIN);  // setup with chip select pin
  adc.setGain(ADS126X_GAIN_32);
  adc.setRate(ADS126X_RATE_38400);
  adc.setFilter(ADS126X_SINC4);
  adc.enableInternalReference();
  adc.startADC1();  // start conversion on ADC1

  adc.disableStatus();
  adc.disableCheck();
  adc.setDelay(ADS126X_DELAY_0);
  adc.clearResetBit();
}

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(USB_SERIAL_BAUDRATE);
  while (!Serial) {
    ;  // Wait for USB serial port to connect. Needed for native USB
  }
#endif  // SERIAL_DEBUG

  // RS485 Serial setup
  SutainaSerial.setupSerial();

  pinMode(CHIP_SELECT_PIN, OUTPUT);

  initADS();

  delay(10);
}

void loop() {
  // dummy read to change pin, see issue
  // https://github.com/Molorius/ADS126X/issues/5
  adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);
  delayMicroseconds(250);

  // actual read
  int32_t reading = adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);

  if (!reading || reading == 1)  // detect if faulty reading occured
  {
    initADS();
  } else if (reading == 0x1000000)  //detect if reset occured
  {
    if (adc.checkResetBit()) {
      initADS();
    }
  } else {
    // sort to get median of previous values
    std::array<int32_t, NUM_AVG> current_previous = previous_values[current_sensor];
    std::sort(current_previous.begin(), current_previous.end());

    // check if current reading deviates too strongly from median
    if (abs(current_previous[NUM_AVG / 2] - reading) < 1e7) {
      force[current_sensor] = reading;
    }
    // update previous values for median calculation
    previous_values[current_sensor][_count] = reading;

    // update current sensor
    current_sensor = (current_sensor + 1) % 4;

    // update counter for median calculating
    if (!current_sensor) {
      _count = (_count + 1) % NUM_AVG;
    }
  }

  if (SutainaSerial.readPacket()) {
    if (SutainaSerial.checkCRCandID()) {
      if (!SutainaSerial.commandProcess()) {
        switch (SutainaSerial.rxCommand) {
          case SEND_VAL_CMD:
            for (const int32_t &val : force) {
              SutainaSerial.txData.push_back(val & 0xFF);
              SutainaSerial.txData.push_back((val >> 8) & 0xFF);
              SutainaSerial.txData.push_back((val >> 16) & 0xFF);
              SutainaSerial.txData.push_back((val >> 24) & 0xFF);
            }

          default:
            SutainaSerial.txError |= SutainaSerial.NOT_EXISTENT_CMD_ERROR;
        }
      }
      SutainaSerial.sendPacket();
    }
  }
}