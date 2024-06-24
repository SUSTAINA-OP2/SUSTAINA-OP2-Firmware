//! Firmware for Power-Logging-Board in SUSTAINA-OP2

#include "./src/INA226/INA226.h"
#include "./src/BitOperations/crc16.hpp"
#include "./src/SdFat/SdFat.h"
#include <vector>
#include <set>
#include <unordered_map>
#include "Wire.h"

#include <ctime>

#define DEBUG 1

#ifdef DEBUG
  // Serial output for debugging PLB( = Power-Logging-Board). 
  #define PLB_DEBUG_SERIAL_PRINTF(...) Serial.printf(__VA_ARGS__);
  #define PLB_DEBUG_SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__);
#else //! DEBUG
  #define PLB_DEBUG_SERIAL_PRINTF(...) ;
  #define PLB_DEBUG_SERIAL_PRINTLN(...) ;
#endif //! DEBUG

/**
   setting by user
*/
constexpr uint8_t FIRMWARE_VERSION = 0x00; // Last updated: 2024/06/xx

/**
   Power-Logging-Board dependent settings
*/

// URAT settings
constexpr uint8_t RX_PIN = 0;
constexpr uint8_t TX_PIN = 1;
constexpr uint8_t TXDEN_PIN = 2;

constexpr uint8_t BOARD_ID = 0xA2;

//! SD Card settings
constexpr uint8_t SD_CS_PIN = 10;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(24))

/**
    not need to change
*/
constexpr uint8_t HEADER_PACKET[] = {0xFE, 0xFE};

constexpr uint32_t USB_SERIAL_BAUDRATE = 9600;
constexpr uint32_t RS485_SERIAL_BAUDRATE = 1000000;

constexpr uint32_t I2C_CLOCK = 400000L;

constexpr uint8_t RETURN_CMD_MASK = 0b01111111;

//! command
constexpr uint8_t READ_VOLTAGE_CURRENT_CMD = 0xC0;
constexpr uint8_t CHECK_SDCARD_CAPACITY_CMD = 0xC1;
constexpr uint8_t RESCAN_I2C_CMD = 0xC2;
constexpr uint8_t TIME_SET_CMD = 0xC3;
constexpr uint8_t SETUP_BIAS_CMD = 0xC4;
constexpr uint8_t BOARD_RESET_CMD = 0xC5;
// constexpr uint8_t CHEACK_FIRMWARE_CMD = 0xD0;

//! error status
/**
    NO_ERROR                    = 0b0000'0000,
    CRC_ERROR                   = 0b0000'0001,
    CMD_NOTFOUND_ERROR          = 0b0000'0010,
    CMD_PROCESSING_ERROR        = 0b0000'0100,

    BOARD_SPECIFIC_ERROR1       = 0b0000'1000, // sdcard error
    BOARD_SPECIFIC_ERROR2       = 0b0001'0000,
    BOARD_SPECIFIC_ERROR3       = 0b0010'0000,
    BOARD_SPECIFIC_ERROR4       = 0b0100'0000,
    BOARD_SPECIFIC_ERROR5       = 0b1000'0000,
*/
constexpr uint8_t CRC_ERROR = 0b00000001;
constexpr uint8_t CMD_UNSUPPORT_ERROR = 0b00000010;
// constexpr uint8_t CMD_PROCESSING_ERROR = 0b00000100;
constexpr uint8_t SDCARD_ERROR = 0b00001000;

constexpr size_t HEADER_PACKET_LENGTH = sizeof(HEADER_PACKET);
constexpr size_t CRC_LENGTH = sizeof(uint16_t);
constexpr size_t FLOAT_LENGTH = sizeof(float);
constexpr size_t ADDRESS_LENGTH = sizeof(uint8_t);

//! rx packet: headder + (BOARD_ID + length + command + option ) + data * n + crc
constexpr size_t RX_PACKET_FORWARD_LENGTH = HEADER_PACKET_LENGTH + 4;
constexpr size_t RX_PACKET_MIN_LENGTH = RX_PACKET_FORWARD_LENGTH + CRC_LENGTH;

//! tx packet: headder + (BOARD_ID + length + command(mask)  + error) + txData + crc
constexpr size_t TX_PACKET_MIN_LENGTH = HEADER_PACKET_LENGTH + 4 + CRC_LENGTH;

//! INA226
constexpr uint8_t LOW_LIMIT_ADDRESS = 0b1000000;   //! 0x40
constexpr uint8_t UPPER_LIMIT_ADDRESS = 0b1001111; //! 0x4F
constexpr size_t INA226_MAX_NUM = UPPER_LIMIT_ADDRESS - LOW_LIMIT_ADDRESS + 1;

//! get time for jetson
constexpr size_t JETSON_SECONDS_TIME_LENGTH = 4;
constexpr size_t JETSON_MILL_TIME_LENGTH = 2;

//! SD Card settings
unsigned long time_data = 0;

bool is_sdcard_error = false;
bool is_send_data = false;

union uint8_tToUint32_t
{
  uint8_t uint8_tData[4];
  uint32_t uint32_tData;
};
union uint8_tToUint16_t
{
  uint8_t uint8_tData[2];
  uint16_t uint16_tData;
};
union uint8_tToFloat
{
  uint8_t uint8_tData[4];
  float floatData;
};

struct INA226BiasData
{
private:
  uint8_t address;
  float voltage;
  float current;

public:
  INA226BiasData() = default;
  INA226BiasData(uint8_t address, float voltage, float current)
      : address(address), voltage(voltage), current(current) {};
  ~INA226BiasData() {};
  void setAddress(const uint8_t &address)
  {
    this->address = address;
  }
  void setVoltage(const float &voltage)
  {
    this->voltage = voltage;
  }
  void setCurrent(const float &current)
  {
    this->current = current;
  }
  uint8_t getAddress()
  {
    return address;
  }
  float getVoltage()
  {
    return voltage;
  }
  float getCurrent()
  {
    return current;
  }
  void setBiasData(const uint8_t &address, const float &voltage, const float &current)
  {
    setAddress(address);
    setVoltage(voltage);
    setCurrent(current);
  }
  void setBiasData(INA226BiasData &bias_data)
  {
    setAddress(bias_data.getAddress());
    setVoltage(bias_data.getVoltage());
    setCurrent(bias_data.getCurrent());
  }
};

class TimeManager
{
public:
  TimeManager() = default;
  ~TimeManager() {};
  std::string getTime()
  {
    if (!is_set_time_)
    {
      return "";
    }
    tm *tm_ptr = localtime(&now_time_seconds_);
    std::string buf(100, 0);
    size_t write_size = strftime(const_cast<char *>(buf.data()), buf.size(), "%Y/%m/%d(%A) %H:%M:%S", tm_ptr);
    snprintf(const_cast<char *>(buf.data()) + write_size, buf.size() - write_size, ".%03d", now_time_milliseconds_);
    return buf;
  }

  void timeUpdate()
  {
    if (!is_set_time_)
    {
      return;
    }

    const unsigned long now = millis();
    const unsigned long elapsed_time = now - first_set_time_;
    now_time_milliseconds_ = time_received_from_jetson_milliseconds_ + elapsed_time % 1000;
    now_time_seconds_ = time_received_from_jetson_seconds_ + elapsed_time / 1000 + now_time_milliseconds_ / 1000;
    now_time_milliseconds_ = now_time_milliseconds_ % 1000;
  }

  void setTime(const uint32_t &seconds, const uint16_t &milliseconds)
  {
    first_set_time_ = millis();
    // receive_seconds_ = seconds;
    // receive_milliseconds_ = milliseconds;
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

struct FreqCalculator
{
  uint32_t start_time;
  uint32_t write_count;
  float freq;
  FreqCalculator()
      : start_time(0), write_count(0), freq(0.0f) {};
  void count()
  {
    if (start_time == 0)
    {
      start_time = millis();
    }
    write_count++;
  }

  float getFreq(const uint32_t &now_time)
  {
    if (start_time == 0)
    {
      return 0.0f;
    }
    if (now_time - start_time > 1000)
    {
      freq = (float)write_count / (float)(now_time - start_time) * 1000.0f;
      start_time = now_time;
      write_count = 0;
    }
    return freq;
  }
};

struct Stopwatch
{
  uint32_t start_time;
  Stopwatch()
      : start_time(0) {};
  void start()
  {
    start_time = micros();
  }
  uint32_t getElapsedTime()
  {
    return micros() - start_time;
  }

  void printElapsedTime()
  {
    PLB_DEBUG_SERIAL_PRINTF("Elapsed Time: %d[us]\n", getElapsedTime());
  }
};

//! get address of connected INA226
std::set<uint8_t> readable_Addresses; //! readable INA226 addresses
void I2cScanner()
{
  uint8_t error = 0;
  readable_Addresses.clear();
  for (uint8_t address = LOW_LIMIT_ADDRESS; address < UPPER_LIMIT_ADDRESS; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      readable_Addresses.insert(address);
    }
  }
}

struct alignas(4) Ina226SensorData
{
  uint8_t padding[3]; // For padding. Not used.
  uint8_t address_;
  float voltage_;
  float current_;
  static constexpr size_t byte_size = sizeof(address_) + sizeof(voltage_) + sizeof(current_);
};

struct Ina226MeasurementData
{
  Ina226MeasurementData()
      : measurement_data_()
  {
    clearData();
  }

  std::array<Ina226SensorData, INA226_MAX_NUM> measurement_data_;
  void clearData()
  {
    measurement_data_.fill({0, 0, 0});
  }

  void setData(const uint8_t &address, const float &voltage, const float &current)
  {
    if (address > UPPER_LIMIT_ADDRESS)
    {
      return;
    }
    const size_t index = address - LOW_LIMIT_ADDRESS;
    measurement_data_[index].address_ = address;
    measurement_data_[index].voltage_ = voltage;
    measurement_data_[index].current_ = current;
  }

  void writeDataToBuff(uint8_t *txData)
  {
    size_t packetIndex = 0;
    for (const auto &data : measurement_data_)
    {
      if (data.address_ == 0)
      {
        // packetIndex += Ina226SensorData::byte_size;
        continue;
      }
      memcpy(txData + packetIndex, &data.address_, Ina226SensorData::byte_size);
      packetIndex += Ina226SensorData::byte_size;
    }
  }

  float getCurrentData(const uint8_t &target_address)
  {
    return measurement_data_[target_address - LOW_LIMIT_ADDRESS].current_;
  }

  float getVoltageData(const uint8_t &target_address)
  {
    return measurement_data_[target_address - LOW_LIMIT_ADDRESS].voltage_;
  }
};

std::vector<INA226> INA;
std::vector<float> rxFloatData;

std::array<INA226BiasData, INA226_MAX_NUM> ina226_all_bias_data;
std::unordered_map<uint8_t, INA226BiasData> ina226_detected_bias_data; // Hash : address, Data : INA226BiasData

Ina226MeasurementData measured_data;

size_t txData_length = 0;
std::vector<uint8_t> txData(txData_length);

SdFat sd;
File logData;

TimeManager time_manager;
FreqCalculator freq_calc;
Stopwatch stopwatch;

uint8_tToUint32_t seconds;
uint8_tToUint16_t milliSeconds;

//! setup INA226s
void setupINA226s()
{
  INA.clear();
  for (const auto &address : readable_Addresses) // std::set follows a narrowly defined weak order, so it comes in order.
  {
    INA.emplace_back(address);

    if (INA.back().begin())
    {
      INA.back().setMaxCurrentShunt(38.73, 0.002);
      INA.back().setShuntVoltageConversionTime(4);
      INA.back().setAverage(2);                   // Determine the number of samples of the sensor value. Here the number of samples is 16.
    }
  }
}

void setup()
{
#if DEBUG
  initializeUSBSerial(USB_SERIAL_BAUDRATE);
#endif
  initializeRS485Serial(RS485_SERIAL_BAUDRATE);
  initializeSDcard();

  Wire.begin();
  Wire.setClock(I2C_CLOCK); //! I2C Set clock change 100kHz to 400kHz

  I2cScanner();
  pinMode(TXDEN_PIN, OUTPUT);
}

void loop()
{

  setupINA226s();

  while (true)
  {

    measured_data.clearData();

    txData.clear();
    digitalWrite(TXDEN_PIN, LOW);
    for (auto &ina_sensor : INA)
    {
      if (ina_sensor.begin())
      {
        //! is Connected
        const uint8_t target_address = ina_sensor.getAddress();
        if (ina226_detected_bias_data.find(target_address) == ina226_detected_bias_data.end())          // When no bias is set.
        {
          INA226Error ec;
          const float current_mA = ina_sensor.getCurrent_mA(ec);
          if(ec) continue;        //If error occurs, skip this sensor
          const float voltage_V = ina_sensor.getBusVoltage(ec);
          if(ec) continue;        //If error occurs, skip this sensor
          measured_data.setData(target_address, voltage_V, current_mA);
        }
        else
        {
          INA226Error ec;
          const float current_mA = ina_sensor.getCurrent_mA(ec);
          if(ec) continue;        //If error occurs, skip this sensor
          const float voltage_V = ina_sensor.getBusVoltage(ec);
          if(ec) continue;        //If error occurs, skip this sensor
          measured_data.setData(target_address, voltage_V + ina226_detected_bias_data[target_address].getVoltage(), current_mA + (ina226_detected_bias_data[target_address].getCurrent() * (current_mA / 1000.0f)));
          PLB_DEBUG_SERIAL_PRINTF("Getdata Address: %x, Voltage: %f, Current: %f\n", target_address, ina226_detected_bias_data[target_address].getVoltage(), ina226_detected_bias_data[target_address].getCurrent());
        }
      }
    }
    if (Serial1.available() >= RX_PACKET_MIN_LENGTH)
    {
      uint8_t rxPacket_forward[RX_PACKET_FORWARD_LENGTH] = {};
      uint8_t tx_errorStatus = 0b00000000;
      if (is_sdcard_error == true)
      {
        tx_errorStatus |= SDCARD_ERROR;
      }

      if (checkHeader(HEADER_PACKET, HEADER_PACKET_LENGTH, rxPacket_forward))
      {
        for (int i = HEADER_PACKET_LENGTH; i < RX_PACKET_FORWARD_LENGTH; i++)
        {
          rxPacket_forward[i] = Serial1.read();
        }

        uint8_t rxBoardType = rxPacket_forward[HEADER_PACKET_LENGTH];
        size_t rxPacket_length = rxPacket_forward[HEADER_PACKET_LENGTH + 1];
        uint8_t rxCommand = rxPacket_forward[HEADER_PACKET_LENGTH + 2];
        uint8_t rxOption = rxPacket_forward[HEADER_PACKET_LENGTH + 3];

        //! make rxPaket
        uint8_t rxPacket[rxPacket_length] = {};
        for (int i = 0; i < rxPacket_length; i++)
        {
          if (i < RX_PACKET_FORWARD_LENGTH)
          {
            rxPacket[i] = rxPacket_forward[i];
          }
          else
          {
            rxPacket[i] = Serial1.read();
          }
        }

        if (calcCRC16_XMODEM(rxPacket, rxPacket_length - CRC_LENGTH) == (uint16_t)(rxPacket[rxPacket_length - CRC_LENGTH] << 8) | (uint16_t)(rxPacket[rxPacket_length - CRC_LENGTH + 1]))
        {
          processCommand(rxCommand, &tx_errorStatus, rxPacket);
        }
        else
        {
          tx_errorStatus |= CRC_ERROR;
        }

        //! tx packet: headder + (command + length + error) + txData + crc
        //! data: (address + voldtage + cureent) * n
        size_t txPacket_length = TX_PACKET_MIN_LENGTH + txData_length;

        //! make txPacket
        uint8_t txPacket[txPacket_length] = {};
        size_t packetIndex = 0;

        //! add forward txPacket
        memcpy(txPacket, HEADER_PACKET, HEADER_PACKET_LENGTH);
        packetIndex += HEADER_PACKET_LENGTH;

        txPacket[packetIndex++] = BOARD_ID;
        txPacket[packetIndex++] = (uint8_t)txPacket_length;
        txPacket[packetIndex++] = rxCommand & RETURN_CMD_MASK; //! command
        txPacket[packetIndex++] = tx_errorStatus;              //! error

        //! add txData to txPacket
        if (!txData.empty())
        {
          memcpy(txPacket + packetIndex, txData.data(), txData_length);
          packetIndex += txData_length;
        }

        //! add CRC to txPacket
        uint16_t txCrc = calcCRC16_XMODEM(txPacket, txPacket_length - CRC_LENGTH);
        txPacket[packetIndex++] = lowByte(txCrc);
        txPacket[packetIndex++] = highByte(txCrc);

        // Serial.write(txPacket, txPacket_length);
        RS485SerialSendData(txPacket, packetIndex);
      }
    } // if (Serial.available() ...

    WriteSDcard();

  } // while
} // loop

void processCommand(const uint8_t &CMD, uint8_t *error, const uint8_t rxPacket[])
{
  txData_length = 0;
  switch (CMD)
  {
  case READ_VOLTAGE_CURRENT_CMD:
  {
    /**
     * @brief:
     * @return:
     */

    //! make txPacket
    txData_length = readable_Addresses.size() * (ADDRESS_LENGTH + 2 * FLOAT_LENGTH);
    txData.resize(txData_length);
    size_t packetIndex = 0;

    measured_data.writeDataToBuff(txData.data());
    break;
  }
  case CHECK_SDCARD_CAPACITY_CMD:
  {
    /**
     * @brief:
     * @return:
     */
    // float cardSize =sd.card()->sectorCount() * 0.512;
    uint32_t free = sd.vol()->freeClusterCount() * sd.vol()->sectorsPerCluster() * 0.512;
    uint8_tToUint32_t freeSize;
    freeSize.uint32_tData = free;
    txData_length = sizeof(uint32_t);
    txData.resize(txData_length);
    for (int i = 0; i < txData_length; i++)
    {
      txData[i] = freeSize.uint8_tData[i];
    }
    break;
  }
  case RESCAN_I2C_CMD:
  {
    /**
     * @brief:
     * @return:
     */
    I2cScanner();
    setupINA226s();
    ina226_detected_bias_data.clear();
    size_t detected_ina_num = readable_Addresses.size();
    txData_length = detected_ina_num;
    txData.resize(txData_length);
    size_t tmp_loop_index = 0;
    for (const auto &address : readable_Addresses)
    {
      txData.at(tmp_loop_index) = address;
      tmp_loop_index++;
      for (int j = 0; j < INA226_MAX_NUM; j++)
      {
        if (address == ina226_all_bias_data[j].getAddress())
        {
          ina226_detected_bias_data[address] = ina226_all_bias_data[j];
          break;
        }
      }
    }
    break;
  }
  case TIME_SET_CMD:
  {
    /**
     * @brief:
     * @return:
     */
    for (int i = RX_PACKET_FORWARD_LENGTH, index = 0; i < RX_PACKET_FORWARD_LENGTH + JETSON_SECONDS_TIME_LENGTH; i++, index++)
    {
      seconds.uint8_tData[index] = rxPacket[i];
    }
    for (int i = RX_PACKET_FORWARD_LENGTH + JETSON_SECONDS_TIME_LENGTH, index = 0; i < RX_PACKET_FORWARD_LENGTH + JETSON_SECONDS_TIME_LENGTH + JETSON_MILL_TIME_LENGTH; i++, index++)
    {
      milliSeconds.uint8_tData[index] = rxPacket[i];
    }
    deserializeReceiveTimeData(seconds, milliSeconds);
    break;
  }
  case SETUP_BIAS_CMD:
  {
    /**
     * @brief:
     * @return:
     */
    ina226_detected_bias_data.clear();
    uint8_tToFloat voltage;
    uint8_tToFloat current;
    static const size_t BIAS_DATA_LENGTH = ADDRESS_LENGTH + (FLOAT_LENGTH + FLOAT_LENGTH);
    for (int ina_num = 0; ina_num < INA226_MAX_NUM; ina_num++)
    {
      size_t bias_head_index = RX_PACKET_FORWARD_LENGTH + ina_num * BIAS_DATA_LENGTH;
      uint8_t address = rxPacket[bias_head_index];
      for (int i = bias_head_index + ADDRESS_LENGTH, index = 0; index < FLOAT_LENGTH; i++, index++)
      {
        voltage.uint8_tData[index] = rxPacket[i];
      }
      for (int i = bias_head_index + ADDRESS_LENGTH + FLOAT_LENGTH, index = 0; index < FLOAT_LENGTH; i++, index++)
      {
        current.uint8_tData[index] = rxPacket[i];
      }
      ina226_all_bias_data[ina_num].setBiasData(address, voltage.floatData, current.floatData);
      PLB_DEBUG_SERIAL_PRINTF("Address: %x, Voltage: %f, Current: %f\n", address, ina226_all_bias_data[ina_num].getVoltage(), ina226_all_bias_data[ina_num].getCurrent());
    }

    for (const auto &address : readable_Addresses)
    {
      for (int j = 0; j < INA226_MAX_NUM; j++)
      {
        if (address == ina226_all_bias_data[j].getAddress())
        {
          ina226_detected_bias_data[address].setBiasData(address, ina226_all_bias_data[j].getVoltage(), ina226_all_bias_data[j].getCurrent());
          PLB_DEBUG_SERIAL_PRINTF("Update detected bias address: %x vol: %f cur: %f\n", ina226_detected_bias_data[address].getAddress(),ina226_detected_bias_data[address].getVoltage(),ina226_detected_bias_data[address].getCurrent());
          break;
        }
      }
    }
    break;
  }
  case BOARD_RESET_CMD:
  {
    /**
     * @brief:
     * @return:
     */
    WriteSDcard(); /// write data before reset
    logData.flush();
    software_reset();
    break;
  }
  default:
    *error |= CMD_UNSUPPORT_ERROR;
  }
}

bool checkHeader(const uint8_t HEADER[], const size_t LENGTH, uint8_t packet[])
{
  for (int i = 0; i < LENGTH; i++)
  {
    if (Serial1.read() != HEADER[i])
    {
      return false;
    }
    packet[i] = HEADER[i];
  }
  return true;
}

#if DEBUG
void initializeUSBSerial(const uint32_t &BAUDRATE)
{
  Serial.begin(BAUDRATE);
  while (!Serial)
  {
    ; //! wait for serial port to connect. Needed for native USB
  }
}
#endif

void initializeRS485Serial(const uint32_t &BAUDRATE)
{
  Serial1.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  while (!Serial1)
  {
    ; //! wait for serial port to connect.
  }
}

void RS485SerialSendData(uint8_t *txPacket, const size_t &LENGTH)
{
  digitalWrite(TXDEN_PIN, HIGH);
  Serial1.write(txPacket, LENGTH);
  Serial1.flush();
  digitalWrite(TXDEN_PIN, LOW);
  is_send_data = true;
}

void initializeSDcard()
{
  PLB_DEBUG_SERIAL_PRINTLN(F("Initializing SD card..."));
  if (!sd.begin(SD_CONFIG))
  {
    PLB_DEBUG_SERIAL_PRINTLN(F("Failed to initialize SD card"));
    is_sdcard_error = true;
    return;
  }
  is_sdcard_error = false;

  int fileNumber = 1;   // Starting file number
  char fileName[12];    // Array containing the file name
  bool created = false; // File creation flag

  while (!created)
  {
    sprintf(fileName, "%d.csv", fileNumber); // Generate file names in uppercase.
    if (!sd.exists(fileName))
    {
      // If file does not exist, create a new file
      logData = sd.open(fileName, FILE_WRITE);
      if (logData)
      {
        PLB_DEBUG_SERIAL_PRINTF("Create file --> %s\n", fileName);
        created = true;
        logData.println("Jetson_Time, Arduino_msec, Send_Data, addr40_Voltage, addr40_Current, addr41_Voltage, addr41_Current, addr42_Voltage, addr42_Current, addr43_Voltage, addr43_Current,addr44_Voltage, addr44_Current, addr45_Voltage, addr45_Current, addr46_Voltage, addr46_Current, addr47_Voltage, addr47_Current, addr48_Voltage, addr48_Current, addr49_Voltage, addr49_Current, addr4a_Voltage, addr4a_Current, addr4b_Voltage, addr4b_Current, addr4c_Voltage, addr4c_Current, addr4d_Voltage, addr4d_Current, addr4e_Voltage, addr4e_Current, addr4f_Voltage, addr4f_Current");
      }
      else
      {
        PLB_DEBUG_SERIAL_PRINTF("Failed to create file --> %s\n", fileName);
      }
    }
    fileNumber++;
  }
}

void WriteSDcard()
{
  static bool is_sdcard_write = false;
  static uint32_t cached_size = 0;
  time_data = millis();
  std::string unix_time_data;
  time_manager.timeUpdate();
  if (!is_sdcard_write)
  {
    unix_time_data = time_manager.getTime();
    if (!unix_time_data.empty())
    {
      is_sdcard_write = true;
    }
  }
  if (unix_time_data.length() != 0)
  {
    logData.print(unix_time_data.c_str());
  }

  static char headerStr[32];
  memset(headerStr, 0, sizeof(headerStr));
  int32_t header_write_size = snprintf(headerStr, sizeof(headerStr), ",%d,%d,", time_data, is_send_data);
  if (header_write_size > 0)
  {
    cached_size += header_write_size;
    logData.write(headerStr, header_write_size);
  }

  is_send_data = false;

  static char dataStr[256];
  memset(dataStr, 0, sizeof(dataStr));

  int32_t wrote_size = 0;
  for (uint8_t target_address = LOW_LIMIT_ADDRESS; target_address <= UPPER_LIMIT_ADDRESS; target_address++)
  {
    if (readable_Addresses.find(target_address) != readable_Addresses.end())
    {
      wrote_size += snprintf(dataStr + wrote_size, sizeof(dataStr), "%4.2f,%5.0f,", target_address,
                             measured_data.getVoltageData(target_address), measured_data.getCurrentData(target_address));
    }
    else
    {
      strcat(dataStr, ",,");
      wrote_size += 2;
    }
  }
  dataStr[wrote_size] = '\n';
  ++wrote_size;
  cached_size += wrote_size;
  logData.write(dataStr, wrote_size);
  if (cached_size > 4096) // Flush if more than 4 KB is cached.
  {
    logData.flush();
    cached_size = 0;
  }
}

void deserializeReceiveTimeData(const uint8_tToUint32_t &seconds, const uint8_tToUint16_t &milliSeconds)
{
  time_t secondsData = seconds.uint32_tData;
  time_t milliSecondsData = milliSeconds.uint16_tData;
  time_manager.setTime(secondsData, milliSecondsData);
}

void software_reset()
{
  ESP.restart();
}
