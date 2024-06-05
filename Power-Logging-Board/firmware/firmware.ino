//! Firmware for Power-Logging-Board in SUSTAINA-OP2
#include "./src/INA226/INA226.h"
#include "./src/CRC16/CRC16.h"
#include "./src/SdFat/SdFat.h"
#include <vector>
#include "Wire.h"

#include <ctime>

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

const uint8_t timeSetCommand = 0xC0;

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


//! get time for jetson
const size_t JETSON_SECONDS_TIME_LENGTH = 4;
const size_t JETSON_MILL_TIME_LENGTH = 2;

union uint8_tToUint32_t{
  uint8_t uint8_tData[4];
  uint32_t uint32_tData;
};
union uint8_tToUint16_t{
  uint8_t uint8_tData[2];
  uint16_t uint16_tData;
};

class TimeManager{
  public:
    TimeManager() = default;
    ~TimeManager(){};
    std::string getTime(){
      if(!is_set_time_){
        return "Not set time yet.";
      }
      tm * tm_ptr = localtime(&now_time_seconds_);
      std::string buf(100,0);
      size_t write_size = strftime(const_cast<char*>(buf.data()),buf.size(),"%Y/%m/%d(%A) %H:%M:%S",tm_ptr);
      snprintf( const_cast<char*>(buf.data()) + write_size,buf.size() - write_size,".%03d",now_time_milliseconds_);
      return buf;
    }
    void timeUpdate(){
      if(!is_set_time_){
        return;
      }

      const unsigned long now = millis();
      const unsigned long elapsed_time = now - first_set_time_;
      now_time_milliseconds_ = time_received_from_jetson_milliseconds_ + elapsed_time % 1000;
      now_time_seconds_ = time_received_from_jetson_seconds_ + elapsed_time / 1000 + now_time_milliseconds_ / 1000;
      now_time_milliseconds_ = now_time_milliseconds_ % 1000;
    }
    void setTime(const uint32_t &seconds, const uint16_t &milliseconds){
      first_set_time_ = millis();
      //receive_seconds_ = seconds;
      //receive_milliseconds_ = milliseconds;
      time_received_from_jetson_seconds_ = seconds;
      time_received_from_jetson_milliseconds_ = milliseconds;
      is_set_time_ = true;
    }
  private:
    bool is_set_time_;
    unsigned long first_set_time_;

    time_t now_time_seconds_;
    time_t now_time_milliseconds_;
    time_t time_received_from_jetson_seconds_;
    time_t time_received_from_jetson_milliseconds_;
    
};

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
SdFat sd;
File logData;

TimeManager time_manager;

uint8_tToUint32_t seconds;
uint8_tToUint16_t milliSeconds;



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
          processCommand(rxCommand, &tx_errorStatus, rxPacket);
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
        if(!readable_Addresses.empty() && !txData.empty()){
          memcpy(txPacket + packetIndex, txData.data(), txData_length);
          packetIndex += txData_length;
        }


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

void processCommand(uint8_t command, uint8_t* error, const uint8_t txPacket[]) {
  switch (command) {
    case readVoltageCurrentCommand:
      {
        /**
            * @brief: 
            * @return: 
            */

        //! make txPacket
        txData_length = readable_Addresses.size() * (sizeof(uint8_t) + 2 * sizeof(float));
        txData.resize(txData_length);
        size_t packetIndex = 0;

        for (size_t i = 0; i < readable_Addresses.size(); ++i) {
          txData[packetIndex++] = readable_Addresses.at(i);

          memcpy(txData.data() + packetIndex, &voltageData.at(i), sizeof(float));
          packetIndex += sizeof(float);

          memcpy(txData.data() + packetIndex, &currentData.at(i), sizeof(float));
          packetIndex += sizeof(float);
        }
        break;
      }
    case timeSetCommand:
      {
        /**
            * @brief: 
            * @return: 
            */
          for(int i = rxPacket_forward_length, index = 0; i < rxPacket_forward_length + JETSON_SECONDS_TIME_LENGTH ; i++, index++){
            seconds.uint8_tData[index] = txPacket[i];
          }
          for(int i = rxPacket_forward_length + JETSON_SECONDS_TIME_LENGTH, index = 0; i < rxPacket_forward_length + JETSON_SECONDS_TIME_LENGTH + JETSON_MILL_TIME_LENGTH; i++, index++){
            milliSeconds.uint8_tData[index] = txPacket[i];
          }
          deserializeReceiveTimeData(seconds, milliSeconds);
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
  time_manager.timeUpdate();
  std::string unix_time_data = time_manager.getTime();
  
  //Serial.printf("Time: %s\n", unix_time_data.c_str());
  logData.print(unix_time_data.c_str());
  logData.print(",");
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

void deserializeReceiveTimeData(const uint8_tToUint32_t &seconds, const uint8_tToUint16_t &milliSeconds){
  time_t secondsData = seconds.uint32_tData;
  time_t milliSecondsData = milliSeconds.uint16_tData;
  time_manager.setTime(secondsData,milliSecondsData);
  tm * tm_ptr = localtime(&secondsData);
  char buf[100];
  memset(buf,0,sizeof(buf));
  strftime(buf,sizeof(buf),"%Y/%m/%d(%A) %H:%M:%S",tm_ptr);
  //Serial.printf("Time: %s.%03d\n",buf,milliSecondsData);
}

