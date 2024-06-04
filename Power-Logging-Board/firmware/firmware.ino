//! Firmware for Power-Logging-Board in SUSTAINA-OP2
#include "./src/INA226/INA226.h"
#include "./src/CRC16/CRC16.h"
#include "./src/SdFat/SdFat.h"
#include <vector>
#include "Wire.h"

/**
   settings by users
*/
const uint8_t txdenPin = 2;

const uint8_t headerPacket[] = { 0xFE, 0xFE };
const uint32_t serialBaudrate = 9600;
const uint32_t serial1Baudrate = 1000000;

//! SD Card config
const uint8_t SD_CS_PIN = SS;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(24)) 
unsigned long time_data = 0;

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

// INA226 num
const uint8_t MAX_INA_NUM = 7;
const size_t INA_DATA_LENGTH = MAX_INA_NUM * (sizeof(uint8_t) + 2 * sizeof(float));

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

std::array<uint8_t, INA_DATA_LENGTH> send_ina_data;

CRC16 CRC;
SdFat sd;
File logData;

void setup() {
  initializeSerial(serialBaudrate);
  initializeSerial1(serial1Baudrate);
  initializeSDcard();

  Wire.begin();
  Wire.setClock(400000L);  //! I2C Set clock change 100kHz to 400kHz

  I2cScanner();
  pinMode(txdenPin, OUTPUT);
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

    send_ina_data.fill(0);

    voltageData.clear();
    currentData.clear();

    txData.clear();

    digitalWrite(txdenPin, LOW);
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
    if (Serial1.available() >= rxPacket_min_length) {
      uint8_t rxPacket_forward[rxPacket_forward_length] = {};
      uint8_t tx_errorStatus = 0b00000000;

      if (checkHeader(headerPacket, headerPacket_length, rxPacket_forward)) {
        for (int i = headerPacket_length; i < rxPacket_forward_length; i++) {
          rxPacket_forward[i] = Serial1.read();
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
            rxPacket[i] = Serial1.read();
          }
        }

        if (CRC.checkCrc16(rxPacket, rxPacket_length)) {
          processCommand(rxCommand, &tx_errorStatus);
        } else {
          tx_errorStatus |= crc_errorStatus;
        }

        //! tx packet: headder + (command + length + error) + txData + crc
        //! data: (address + voldtage + cureent) * n
        size_t txPacket_length = txPacket_min_length + INA_DATA_LENGTH;

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
        memcpy(txPacket + packetIndex, send_ina_data.data(), INA_DATA_LENGTH);
        packetIndex += INA_DATA_LENGTH;


        //! add CRC to txPacket
        uint16_t txCrc = CRC.getCrc16(txPacket, txPacket_length - crc_length);
        txPacket[packetIndex++] = lowByte(txCrc);
        txPacket[packetIndex++] = highByte(txCrc);


        //Serial.write(txPacket, txPacket_length);
        serial1SendData(txPacket, packetIndex);
      }
    }  // if (Serial.available() ...

    WriteSDcard();

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
        //txData_length = readable_Addresses.size() * (sizeof(uint8_t) + 2 * sizeof(float));
        //txData.resize(txData_length);
        size_t packetIndex = 0;

        for (size_t i = 0; i < readable_Addresses.size(); ++i) {
          send_ina_data[packetIndex++] = readable_Addresses.at(i);

          memcpy(send_ina_data.data() + packetIndex, &voltageData.at(i), sizeof(float));
          packetIndex += sizeof(float);

          memcpy(send_ina_data.data() + packetIndex, &currentData.at(i), sizeof(float));
          packetIndex += sizeof(float);
        }
        break;
      }

    default:
      *error |= commandUnsupport_errorStatus;
  }
}

bool checkHeader(const uint8_t header[], const size_t length, uint8_t packet[]) {
  for (int i = 0; i < length; i++) {
    if (Serial1.read() != header[i]) {
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
  Serial1.begin(serial1Baudrate, SERIAL_8N1, 0, 1);
  while (!Serial1) {
    ;  //! wait for serial port to connect.
  }
}

void serial1SendData(uint8_t* txPacket, size_t packet_num) {
  digitalWrite(txdenPin, HIGH);
  Serial1.write(txPacket, packet_num);
  delayMicroseconds(1000);
  digitalWrite(txdenPin, LOW);
}

void initializeSDcard(){
  // SDカードのセットアップ
  Serial.println(F("Initializing SD card..."));
  if (!sd.begin(SD_CONFIG))
  {
    Serial.println(F("Failed to initialize SD card"));
    return;
  }

  int fileNumber = 1; // ファイル番号の開始
  char fileName[12]; // ファイル名を格納する配列
  bool created = false; // ファイル作成フラグ

  while (!created) {
    sprintf(fileName, "%d.csv", fileNumber); // ファイル名を大文字で生成
    if (!sd.exists(fileName)) {
      // ファイルが存在しない場合、新しいファイルを作成
      logData = sd.open(fileName, FILE_WRITE);
      if (logData) {
        Serial.print(fileName);
        Serial.println(" を作成しました。");
        created = true; // ファイルを作成したのでフラグを立てる
      } else {
        Serial.print(fileName);
        Serial.println(" の作成に失敗しました。");
      }
    } 
    fileNumber++; // 次の番号へ
  }
}

void WriteSDcard()
{
  time_data = millis();
  logData.print(time_data);
  logData.print(",");

  char dataStr[100] = "";
  char buffer[7];

  for (size_t i = 0; i < readable_Addresses.size(); i++) {

    itoa(readable_Addresses.at(i), buffer, 16);
    strcat(dataStr, buffer);
    strcat(dataStr, ", ");
    
    dtostrf(voltageData.at(i), 5, 1, buffer);
    strcat(dataStr, buffer);
    strcat(dataStr, ", ");
    
    dtostrf(currentData.at(i), 5, 1, buffer);
    strcat(dataStr, buffer);
    strcat(dataStr, ", ");
  
  }

  logData.println(dataStr);
  logData.flush();
}