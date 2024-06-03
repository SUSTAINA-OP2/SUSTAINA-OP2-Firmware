//! Firmware for Power-Logging-Board in SUSTAINA-OP2
#include "./src/INA226/INA226.h"
#include "./src/CRC16/CRC16.h"
#include <vector>
#include "Wire.h"

/**
   settings by users
*/
const uint8_t txdenPin = 2;

const uint8_t headerPacket[] = { 0xFE, 0xFE };
const uint32_t serialBaudrate = 9600;
const uint32_t serial1Baudrate = 1000000;

//! USB-to-Quad-RS-485-Conv-Module:  0xA0 (ID: 160)
//! Control-Switches-Board:          0xA1 (ID: 161)
//! Power Logging Board:             0xA2 (ID: 162)
//! Audio Board:                     0xA3 (ID: 163)
const uint8_t id = 0xA2;

const uint8_t firmwareVersion = 0x00;

//! command
//! commands to return the values: 0xA*
const uint8_t readVoltageCurrentCommand = 0xA0;

//! commands to change/return the values: 0xB*
const uint8_t setupBiasCommand = 0xB0;
const uint8_t cheackFirmwareCommand = 0xD0;

//! error status
const uint8_t crc_errorStatus = 0b00000010;
const uint8_t commandUnsupport_errorStatus = 0b00000010;
const uint8_t commandProcessing_errorStatus = 0b00000100;

/**
   settings users do not need to change
*/
const size_t headerPacket_length = sizeof(headerPacket);
const size_t crc_length = sizeof(uint16_t);

//! rx packet: headder + (id + command + length) + data * n + crc
const size_t rxPacket_forward_length = headerPacket_length + 3;
const size_t rxPacket_min_length = rxPacket_forward_length + crc_length;

//! tx packet: headder + (id + command + length + error) + txData + crc
const size_t txPacket_min_length = headerPacket_length + 4 + crc_length;
const uint8_t lowLimit_Address = 0b1000000;    //! 0x40
const uint8_t upperLimit_Address = 0b1001111;  //! 0x4F


//! get address of connected INA226
std::vector<uint8_t> readable_Addresses;  //! readable INA226 addresses
void I2cScanner() {
  uint8_t error = 0;
  readable_Addresses.clear();
  for (uint8_t address = lowLimit_Address; address < upperLimit_Address; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      readable_Addresses.push_back(address);
    }
  }
}

std::vector<INA226> INA;
std::vector<float> rxFloatData;
std::vector<float> voltageData;
std::vector<float> currentData;

size_t txData_length = 0;
std::vector<uint8_t> txData(txData_length);

CRC16 CRC;

void setup() {
  initializeSerial(serialBaudrate);
  // initializeSerial1(serial1Baudrate);

  Wire.begin();
  Wire.setClock(400000L);  //! I2C Set clock change 100kHz to 400kHz

  I2cScanner();
}

void loop() {

  //! setup INA226s
  for (size_t i = 0; i < readable_Addresses.size(); i++) {
    uint8_t address = readable_Addresses.at(i);
    INA.emplace_back(address);

    if (INA.back().begin()) {
      INA.back().setMaxCurrentShunt(20, 0.002);
    }
  }

  while (true) {

    voltageData.clear();
    currentData.clear();

    txData.clear();

    for (size_t i = 0; i < readable_Addresses.size(); i++) {
      if (INA[i].begin()) {
        //! is Connected
        voltageData.push_back(INA[i].getBusVoltage());
        currentData.push_back(INA[i].getCurrent_mA());
      } else {
        //! is not Connected
        voltageData.push_back(0.0f);
        currentData.push_back(0.0f);
      }
    }

    if (Serial.available() >= rxPacket_min_length) {
      uint8_t rxPacket_forward[rxPacket_forward_length] = {};
      uint8_t tx_errorStatus = 0b00000000;

      if (checkHeader(headerPacket, headerPacket_length, rxPacket_forward)) {

        for (int i = headerPacket_length; i < rxPacket_forward_length; i++) {
          rxPacket_forward[i] = Serial.read();
        }

        uint8_t rxBoardType = rxPacket_forward[headerPacket_length];
        uint8_t rxCommand = rxPacket_forward[headerPacket_length + 1];
        size_t rxPacket_length = rxPacket_forward[headerPacket_length + 2];

        //! make rxPaket
        uint8_t rxPacket[rxPacket_length] = {};
        for (int i = 0; i < rxPacket_length; i++) {
          if (i < rxPacket_forward_length) {
            rxPacket[i] = rxPacket_forward[i];
          } else {
            rxPacket[i] = Serial.read();
          }
        }

        if (CRC.checkCrc16(rxPacket, rxPacket_length)) {
          processCommand(rxCommand, &tx_errorStatus);
        } else {
          tx_errorStatus |= crc_errorStatus;
        }

        //! tx packet: headder + (command + length + error) + txData + crc
        //! data: (address + voldtage + cureent) * n
        size_t txPacket_length = txPacket_min_length + txData_length;

        //! make txPacket
        uint8_t txPacket[txPacket_length] = {};
        size_t packetIndex = 0;

        //! add forward txPacket
        memcpy(txPacket, headerPacket, headerPacket_length);
        packetIndex += headerPacket_length;
        txPacket[packetIndex++] = id;
        txPacket[packetIndex++] = rxCommand;  //! command
        txPacket[packetIndex++] = (uint8_t)txPacket_length;
        txPacket[packetIndex++] = tx_errorStatus;  //! error

        //! add txData to txPacket
        if (!readable_Addresses.empty()) {
          memcpy(txPacket + packetIndex, txData.data(), txData_length);
          packetIndex += txData_length;
        }

        //! add CRC to txPacket
        uint16_t txCrc = CRC.getCrc16(txPacket, txPacket_length - crc_length);
        txPacket[packetIndex++] = lowByte(txCrc);
        txPacket[packetIndex++] = highByte(txCrc);

        Serial.write(txPacket, txPacket_length);
        // serial1SendData(txPacket);
      }
    }  // if (Serial.available() ...
  }    // while
}  // loop

void processCommand(uint8_t command, uint8_t* error) {
  switch (command) {
    case readVoltageCurrentCommand:
      {
        /**
            * @brief: 
            * @return: 
            */

        //! make txPacket
        txData_length = readable_Addresses.size() * (sizeof(uint8_t) + 2 * sizeof(float));

        size_t packetIndex = 0;

        for (size_t i = 0; i < readable_Addresses.size(); ++i) {
          txData[packetIndex++] = readable_Addresses[i];

          memcpy(txData.data() + packetIndex, &voltageData[i], sizeof(float));
          packetIndex += sizeof(float);

          memcpy(txData.data() + packetIndex, &currentData[i], sizeof(float));
          packetIndex += sizeof(float);
        }
      }

    default:
      *error |= commandUnsupport_errorStatus;
  }
}

bool checkHeader(const uint8_t header[], const size_t length, uint8_t packet[]) {
  for (int i = 0; i < length; i++) {
    if (Serial.read() != header[i]) {
      return false;
    }
    packet[i] = header[i];
  }
  return true;
}

void initializeSerial(uint32_t serialBaudrate) {
  Serial.begin(serialBaudrate);
  while (!Serial) {
    ;  //! wait for serial port to connect. Needed for native USB
  }
}

void initializeSerial1(uint32_t serial1Baudrate) {
  Serial1.begin(serial1Baudrate);
  while (!Serial1) {
    ;  //! wait for serial port to connect.
  }
}

void serial1SendData(uint8_t* txPacket) {
  digitalWrite(txdenPin, HIGH);
  Serial1.write(txPacket, sizeof(txPacket));
  delayMicroseconds(1000);
  digitalWrite(txdenPin, LOW);
}
