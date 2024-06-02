/*
実装する機能のリスト

・ SDカードに記録するcsvに Arduinoのシステム時間・Jetsonのシステム時間を加える
  ・ Jetsonの方は取得するのに時間がかかるので、取得できないときは空白にする

*/

#include "./src/SdFat/SdFat.h"
#include "./src/INA226/INA226.h"
#include "./src/CRC16/CRC16.h"
#include <vector>
#include <time.h>

const uint8_t SD_CS_PIN = SS;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(24))

/**
 * settings by users
 */

const uint8_t headerPacket[] = { 0xFE, 0xFE };
//const uint32_t UsbSerial_baudrate = 115200;
const uint32_t UsbSerial_baudrate = 1000000;
const uint8_t firmwareVersion = 0x01;

//! command
//! commands to return the values: 0xA*
const uint8_t ReadVoltageCurrentCommand = 0xA0;

//! commands to change/return the values: 0xB*
const uint8_t setupBiasCommand = 0xB0;

const uint8_t cheackFirmwareCommand = 0xD0;

//! error status
const uint8_t crc_errorStatus = 0b00000010;
const uint8_t commandUnsupport_errorStatus = 0b00010000;
const uint8_t commandProcessing_errorStatus = 0b00100000;

/**
 * settings users do not need to change
 */

const size_t headerPacket_length = sizeof(headerPacket);
const size_t crc_length = sizeof(uint16_t);

//! rx packet: headder + command + length + data * n + crc
const size_t rxPacket_forward_length = headerPacket_length + 2;
const size_t rxPacket_min_length = rxPacket_forward_length + crc_length;

//! tx packet: headder + command + length + error + data * n + crc
const size_t txPacket_min_length = headerPacket_length + 3 + crc_length;

std::vector<float> rxFloatData;
std::vector<float> txFloatData;
std::vector<uint8_t> txByteData;
std::vector<float> sensorData;

unsigned long ArduinoTimeStamp;

float _INA226Bias[6][3] = {};

CRC16 CRC;

SdFat sd;
File logData;

INA226 address_0x40(0x40);
bool address_0x40_OK = false;

INA226 address_0x41(0x41);
bool address_0x41_OK = false;

INA226 address_0x42(0x42);
bool address_0x42_OK = false;

INA226 address_0x43(0x43);
bool address_0x43_OK = false;

INA226 address_0x44(0x44);
bool address_0x44_OK = false;

INA226 address_0x45(0x45);
bool address_0x45_OK = false;

void ReadSensorData()
{

  if(address_0x40_OK)
  {
    sensorData.push_back(_INA226Bias[0][0]);
    sensorData.push_back(address_0x40.getBusVoltage() + _INA226Bias[0][1]);
    sensorData.push_back(address_0x40.getCurrent_mA() + _INA226Bias[0][2]);
  }
  if(address_0x41_OK)
  {
    sensorData.push_back(_INA226Bias[1][0]);
    sensorData.push_back(address_0x41.getBusVoltage() + _INA226Bias[1][1]);
    sensorData.push_back(address_0x41.getCurrent_mA() + _INA226Bias[1][2]);
  }
  if(address_0x42_OK)
  {
    sensorData.push_back(_INA226Bias[2][0]);
    sensorData.push_back(address_0x42.getBusVoltage() + _INA226Bias[2][1]);
    sensorData.push_back(address_0x42.getCurrent_mA() + _INA226Bias[2][2]);
  }
  if(address_0x43_OK)
  {
    sensorData.push_back(_INA226Bias[3][0]);
    sensorData.push_back(address_0x43.getBusVoltage() + _INA226Bias[3][1]);
    sensorData.push_back(address_0x43.getCurrent_mA() + _INA226Bias[3][2]);
  }
  if(address_0x44_OK)
  {
    sensorData.push_back(_INA226Bias[4][0]);
    sensorData.push_back(address_0x44.getBusVoltage() + _INA226Bias[4][1]);
    sensorData.push_back(address_0x44.getCurrent_mA() + _INA226Bias[4][2]);
  } 
  if(address_0x45_OK)
  {
    sensorData.push_back(_INA226Bias[5][0]);
    sensorData.push_back(address_0x45.getBusVoltage() + _INA226Bias[5][1]);
    sensorData.push_back(address_0x45.getCurrent_mA() + _INA226Bias[5][2]);
  }
}

void WriteSDcard()
{

  ArduinoTimeStamp = millis();

  logData.print(ArduinoTimeStamp);
  logData.print(",");

  char dataStr[100] = "";
  char buffer[7];

  for (int i = 0; i < sensorData.size(); i++){
    dtostrf(sensorData[i], 5, 1, buffer);
    strcat(dataStr, buffer);
    strcat(dataStr, ", ");
  }

  logData.println(dataStr);
  logData.flush();
}

void SendData()
{
  for (int i = 0; i < sensorData.size(); i++) {
  txFloatData.push_back(sensorData[i]);
  }
}

void setup() 
{
  initializeSerial(UsbSerial_baudrate);
  initializeINA226();
  initializeSDcard();
  Serial.println("initialize success");
}

void loop() {

  sensorData.clear();

  ReadSensorData();
  WriteSDcard();

  if (Serial.available() >= rxPacket_min_length) 
  {
    uint8_t rxPacket_forward[rxPacket_forward_length] = {};

    if (checkHeader(headerPacket, headerPacket_length, rxPacket_forward)) {

      for (int i = headerPacket_length; i < rxPacket_forward_length; i++) {
        rxPacket_forward[i] = Serial.read();
      }

      uint8_t rxCommand = rxPacket_forward[headerPacket_length];
      size_t rxPacket_length = rxPacket_forward[headerPacket_length + 1];

      uint8_t rxPacket[rxPacket_length] = {};

      for (int i = 0; i < rxPacket_length; i++) {
        if (i < rxPacket_forward_length) {
          rxPacket[i] = rxPacket_forward[i];
        } else {
          rxPacket[i] = Serial.read();
        }
      }

      uint8_t tx_errorStatus = 0b00000000;

      rxFloatData.clear();
      txFloatData.clear();
      txByteData.clear();

      size_t txFloatData_length = 0;
      size_t txByteData_length = 0;
      size_t txPacket_length = txPacket_min_length;


      if (CRC.checkCrc16(rxPacket, rxPacket_length)) {

        if (rxPacket_length > rxPacket_min_length) {

          size_t rxFloatData_length = rxPacket_length - rxPacket_min_length;

          uint8_t rxFloatData_bytes[rxFloatData_length] = {};
          for (int i = 0; i < rxFloatData_length; i++) {
            rxFloatData_bytes[i] = rxPacket[i + rxPacket_forward_length];
          }

          size_t rxFloatData_count = rxFloatData_length / sizeof(float);

          for (int i = 0; i < rxFloatData_count; i++) {
            float rxFloatData_float = 0;
            uint8_t* bytePtr = (uint8_t*)&rxFloatData_float;

            for (int j = 0; j < sizeof(float); j++) {
              bytePtr[j] = rxFloatData_bytes[i * sizeof(float) + j];
            }

            rxFloatData.push_back(rxFloatData_float);
          }
        }

        processCommand(rxCommand, &tx_errorStatus);

        if (!txFloatData.empty()) {
          txFloatData_length = txFloatData.size() * sizeof(float);
          txPacket_length += txFloatData_length;
        }

        if (!txByteData.empty()) {
          txByteData_length = txByteData.size() * sizeof(uint8_t);
          txPacket_length += txByteData_length;
        }
      } else {
        tx_errorStatus |= crc_errorStatus;
      }


      uint8_t txPacket[txPacket_length] = {};
      size_t packetIndex = 0;

      memcpy(txPacket, headerPacket, headerPacket_length);
      packetIndex += headerPacket_length;

      txPacket[packetIndex++] = rxCommand;
      txPacket[packetIndex++] = (uint8_t)txPacket_length;
      txPacket[packetIndex++] = tx_errorStatus;

      if (!txFloatData.empty()) {
        memcpy(txPacket + packetIndex, txFloatData.data(), txFloatData_length);
        packetIndex += txFloatData_length;
      }

      if (!txByteData.empty()) {
        memcpy(txPacket + packetIndex, txByteData.data(), txByteData_length);
        packetIndex += txByteData_length;
      }

      uint16_t txCrc = CRC.getCrc16(txPacket, txPacket_length - crc_length);
      txPacket[packetIndex++] = lowByte(txCrc);
      txPacket[packetIndex++] = highByte(txCrc);

      Serial.write(txPacket, txPacket_length);

      if (tx_errorStatus != 0b00000000) {
        digitalWrite(13, LOW);  // light on
      } else {
        digitalWrite(13, HIGH);  //light on
      }
    }
  }
}

void processCommand(uint8_t command, uint8_t* error) {

  switch (command) {
      case ReadVoltageCurrentCommand:
        SendData();
        break;

      case setupBiasCommand:
        {
        if (rxFloatData.empty() || rxFloatData.size() != 18) {
          *error |= commandProcessing_errorStatus;
          break;
        }

        int count = 0;

        for (int i = 0; i < 6; i++){
          for (int j = 0; j < 3; j++){
            _INA226Bias[i][j] = rxFloatData[count];
            count++;
          }
        }

        for (int i = 0; i < 18; i++){
          txFloatData.push_back(rxFloatData.at(i));
        }

        break;
        }

      case cheackFirmwareCommand:
        /**
              * @brief:
              * @return: firmware virsion
              */
        txByteData.push_back(firmwareVersion);
        break;

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

void initializeSerial(uint32_t baudrate) {
  Serial.begin(baudrate);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }
}

void initializeINA226(){
  // I2Cの設定clockを100kHzから400Hzに変更して、取得速度を上げている
  Wire.begin();
  Wire.setClock(400000L); // I2C Set clock change 100kHz to 400kHz

  // Device initialization
  if (!address_0x40.begin() )
  {
    Serial.println("could not connect. address_0x40");
    address_0x40_OK = false;
  }else
  {
    address_0x40.setMaxCurrentShunt(1, 0.002);
    address_0x40_OK = true;
  }

  if (!address_0x41.begin() )
  {
    Serial.println("could not connect. address_0x41");
    address_0x41_OK = false;
  }else
  {
    address_0x41.setMaxCurrentShunt(1, 0.002);
    address_0x41_OK = true;
  }

  if (!address_0x42.begin() )
  {
    Serial.println("could not connect. address_0x42");
    address_0x42_OK = false;
  }else
  {
    address_0x42.setMaxCurrentShunt(1, 0.002);
    address_0x42_OK = true;
  }

  if (!address_0x43.begin() )
  {
    Serial.println("could not connect. address_0x43");
    address_0x43_OK = false;
  }else
  {
    address_0x43.setMaxCurrentShunt(1, 0.002);
    address_0x43_OK = true;
  }

  if (!address_0x44.begin() )
  {
    Serial.println("could not connect. address_0x44");
    address_0x44_OK = false;
  }else
  {
    address_0x44.setMaxCurrentShunt(1, 0.002);
    address_0x44_OK = true;
  }

  if (!address_0x45.begin() )
  {
    Serial.println("could not connect. address_0x45");
    address_0x45_OK = false;
  }else
  {
    address_0x45.setMaxCurrentShunt(1, 0.002);
    address_0x45_OK = true;
  }

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
