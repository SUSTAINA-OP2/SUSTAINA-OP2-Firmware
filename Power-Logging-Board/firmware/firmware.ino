//! Firmware for Power-Logging-Board in SUSTAINA-OP2
#include "./src/INA226/INA226.h"
#include "./src/CRC16/CRC16.h"
#include "./src/SdFat/SdFat.h"
#include <vector>
#include <set>
#include <unordered_map>
#include "Wire.h"

#include <ctime>

#define DEBUG 1

// freeRTOS settings.
#define INCLUDE_vTaskSuspend 1

/**
   settings by users
*/
constexpr uint8_t txdenPin = 2;

constexpr uint8_t headerPacket[] = {0xFE, 0xFE};
constexpr uint32_t serialBaudrate = 9600;
constexpr uint32_t serial1Baudrate = 1000000;

//! SD Card config
constexpr uint8_t SD_CS_PIN = SS;
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(24))
unsigned long time_data = 0;

//! USB-to-Quad-RS-485-Conv-Module:  0xA0 (ID: 160)
//! Control-Switches-Board:          0xA1 (ID: 161)
//! Power Logging Board:             0xA2 (ID: 162)
//! Audio Board:                     0xA3 (ID: 163)
constexpr uint8_t id = 0xA2;

constexpr uint8_t firmwareVersion = 0x00;

//! command
//! commands to return the values: 0xA*
constexpr uint8_t readVoltageCurrentCommand = 0xC0;

//! commands to change/return the values: 0xB*
constexpr uint8_t cheackFirmwareCommand = 0xD0;

constexpr uint8_t checkSDcardCapacityCommand = 0xC1;
constexpr uint8_t rescanI2CCommand = 0xC2;
constexpr uint8_t timeSetCommand = 0xC3;

constexpr uint8_t setupBiasCommand = 0xC4;
constexpr uint8_t boardResetCommand = 0xC5;

//! error status
/**
    NO_ERROR                    = 0b0000'0000,
    CRC_ERROR                   = 0b0000'0001,
    COMMAND_NOTFOUND_ERROR      = 0b0000'0010,
    COMMAND_PROCESSING_ERROR    = 0b0000'0100,

    BOARD_SPECIFIC_ERROR1       = 0b0000'1000, // sdcard error
    BOARD_SPECIFIC_ERROR2       = 0b0001'0000,
    BOARD_SPECIFIC_ERROR3       = 0b0010'0000,
    BOARD_SPECIFIC_ERROR4       = 0b0100'0000,
    BOARD_SPECIFIC_ERROR5       = 0b1000'0000,
*/
constexpr uint8_t crc_errorStatus = 0b00000001;
constexpr uint8_t commandUnsupport_errorStatus = 0b00000010;
constexpr uint8_t commandProcessing_errorStatus = 0b00000100;
constexpr uint8_t sdcard_errorStatus = 0b00001000;

constexpr uint8_t return_command_mask = 0b01111111;

/**
   settings users do not need to change
*/
constexpr size_t headerPacket_length = sizeof(headerPacket);
constexpr size_t crc_length = sizeof(uint16_t);

//! rx packet: headder + (id + length + command + option ) + data * n + crc
constexpr size_t rxPacket_forward_length = headerPacket_length + 4;
constexpr size_t rxPacket_min_length = rxPacket_forward_length + crc_length;

//! tx packet: headder + (id + command + length + error) + txData + crc
constexpr size_t txPacket_min_length = headerPacket_length + 4 + crc_length;
constexpr uint8_t lowLimit_Address = 0b1000000;   //! 0x40
constexpr uint8_t upperLimit_Address = 0b1001111; //! 0x4F

//! get time for jetson
constexpr size_t JETSON_SECONDS_TIME_LENGTH = 4;
constexpr size_t JETSON_MILL_TIME_LENGTH = 2;

constexpr size_t INA226_MAX_NUM = 16;

constexpr size_t FLOAT_DATA_LENGTH = sizeof(float);

const char *filename_for_sdcard_exists = "does_sdcard_exists.csv";
void initializeSDcard(); // forward declaration

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
  INA226BiasData(uint8_t address, float voltage, float current) : address(address), voltage(voltage), current(current) {};
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
  FreqCalculator() : start_time(0), write_count(0), freq(0.0f) {};
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
  Stopwatch() : start_time(0) {};
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
    Serial.printf("Elapsed Time: %d[us]\n", getElapsedTime());
  }
};

// ----------------------- SDcard task ---------------------------------
volatile SemaphoreHandle_t shared_data_semaphore = NULL;

struct SDWriter
{
  SdFat *sd_;
  File *logData_;
  static constexpr size_t sd_write_buffer_size_ = 8192;
  static constexpr size_t data_buffer_size_ = 8192; // 4096 + 1024
  char sd_write_buffer_[sd_write_buffer_size_];
  char data_buffer_[data_buffer_size_];
  size_t current_data_size_;
  bool sd_card_not_exist_ = false;

  SDWriter() : sd_(NULL), logData_(NULL), current_data_size_(0)
  {
    memset(sd_write_buffer_, 0, sizeof(sd_write_buffer_));
    memset(data_buffer_, 0, sizeof(data_buffer_));
  };

  void setSDcard(SdFat *sd, File *logData)
  {
    // 全てのロックを取得
    if (shared_data_semaphore == NULL)
    {
      shared_data_semaphore = xSemaphoreCreateMutex();
    }
    while (true)
    {
      if (xSemaphoreTake(shared_data_semaphore, portMAX_DELAY) == pdTRUE) //    ------------------------------------- Lock Acuqire
      {
        sd_ = sd;
        logData_ = logData;
        xSemaphoreGive(shared_data_semaphore); //    ------------------------------------- Lock Release
        break;
      }
    }
  }

  void addData(const char *data, const size_t &data_sizes)
  {
    if ((data_sizes > data_buffer_size_) || (sd_ == NULL) || (logData_ == NULL) || (shared_data_semaphore == NULL) || sd_card_not_exist_)
    {
      return;
    }
    if ((current_data_size_ + data_sizes) >= data_buffer_size_)
    {
      return;
    }
    if (xSemaphoreTake(shared_data_semaphore, portMAX_DELAY) == pdTRUE) //    ------------------------------------- Lock Acuqire
    {
      strncpy(data_buffer_ + current_data_size_, data, data_sizes);
      current_data_size_ += data_sizes;
      xSemaphoreGive(shared_data_semaphore); //    ------------------------------------- Lock Release
      // Serial.printf("[[addData]] data_sizes = %d\n",data_sizes);
    }
    return;
  }

  /**
   * @brief Acquire lock immediately and write data to buffer
   *
   * @param data const char* string data
   */
  void println(const char *data)
  {
    if ((sd_ == NULL) || (logData_ == NULL) || (shared_data_semaphore == NULL))
    {
      return;
    }
    if ((current_data_size_ + strlen(data)) >= data_buffer_size_)
    {
      return;
    }
    if (xSemaphoreTake(shared_data_semaphore, portMAX_DELAY) == pdTRUE) //    ------------------------------------- Lock Acuqire
    {
      strncpy(data_buffer_ + current_data_size_, data, strlen(data));
      current_data_size_ += strlen(data);
      xSemaphoreGive(shared_data_semaphore); //    ------------------------------------- Lock Release
    }
    return;
  }

  /**
   * @brief Check if the SD card is available.
   * @return true sd card is available.
   * @return false sd card is not available.
   * @note If the SD card is not available, the SD card is reconnected.
   *       You must get the lock before calling this function.
   */
  bool checkSDCardAvailable()
  {
    if (sd_ == NULL && logData_ == NULL)
    {
      initializeSDcard(); // SDカードが一番最初にセットアップされていない時
      sd_card_not_exist_ = true;
      return !sd_card_not_exist_;
    }
    if (!sd_->exists(filename_for_sdcard_exists))
    {
      sd_card_not_exist_ = true;
      if (sd_card_not_exist_) // SDカードが無い状態 -> ある状態に変わった時
      {
        Serial.println("Start reconnecting SD card...");
        sd_->end();         // SDカードを終了
        initializeSDcard(); // SDカードを再度セットアップ
        Serial.println("SD card is reconnected");
        sd_card_not_exist_ = false;
      }
      return !sd_card_not_exist_;
    }
    return !sd_card_not_exist_;
  }

  /**
   * @brief Check if the SD card is available.
   *
   * @return true sd card is available.
   * @return false sd card is not available.
   * @note This function only checks the flag.If you want to check the SD card, use checkSDCardAvailable().
   */
  bool getSDCardAvailableFlag() const
  {
    return !sd_card_not_exist_;
  }

  /**
   * @brief
   * @return true Flush was executed.
   * @return false Flush was not executed.
   */
  bool flush()
  {
    // セマフォを取る
    memset(sd_write_buffer_, 0, sizeof(sd_write_buffer_));
    if (xSemaphoreTake(shared_data_semaphore, portMAX_DELAY) == pdTRUE) //    ------------------------------------- Lock Acuqire
    {
      if (!checkSDCardAvailable())
      {
        Serial.println("SD card is not available");
        xSemaphoreGive(shared_data_semaphore); //    ------------------------------------- Lock Release
        return false;
      }
      if (current_data_size_ < (sd_write_buffer_size_ - 500)) // あまり小さいサイズの時は書き込まない
      {
        xSemaphoreGive(shared_data_semaphore); //    ------------------------------------- Lock Release
        return false;
      }
      memcpy(sd_write_buffer_, data_buffer_, current_data_size_); // データを書き込み用バッファにコピー
      const size_t write_size = current_data_size_;
      current_data_size_ = 0;
      memset(data_buffer_, 0, sizeof(data_buffer_));         // データバッファをクリア
      xSemaphoreGive(shared_data_semaphore);                 //    ------------------------------------- Lock Release
      logData_->write(sd_write_buffer_, write_size);         // 書き込み
      memset(sd_write_buffer_, 0, sizeof(sd_write_buffer_)); // 書き込み用バッファをクリア
      logData_->flush();                                     // フラッシュ
      return true;
    }
    return false;
  }
};

SDWriter sd_writer;

void sdWriterTask(void *pvParameters)
{
  Serial.println("SDWriterTask Start");
  while (true)
  {
    if (shared_data_semaphore != NULL)
    {
      if (sd_writer.flush())
      {
        vTaskDelay(10 / portTICK_PERIOD_MS); // 10msec毎にチェック
      }
      else
      {
        vTaskDelay(5 / portTICK_PERIOD_MS); // 5msec待つ
      }
    }
    else
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

/**
 * @brief Set the Up SD Write Task object.
 *        This must be called last in setup() function.
 */
void setUpSDWriteTask()
{
  shared_data_semaphore = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sdWriterTask, "sdWriterTask", 4096, NULL, 1, NULL, 0); // core 0 is another core of the loop()
}

// ----------------------- ---------------------------------

//! get address of connected INA226
std::set<uint8_t> readable_Addresses; //! readable INA226 addresses
void I2cScanner()
{
  uint8_t error = 0;
  readable_Addresses.clear();
  for (uint8_t address = lowLimit_Address; address < upperLimit_Address; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%x\n", address);
      readable_Addresses.insert(address);
    }
  }
}

struct alignas(4) Ina226SensorData
{
  uint8_t padding[3]; // いい感じにパディングする
  uint8_t address_;
  float voltage_;
  float current_;
  static constexpr size_t byte_size = sizeof(address_) + sizeof(voltage_) + sizeof(current_);
};

struct Ina226MeasurementData
{
  Ina226MeasurementData() : measurement_data_()
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
    if (address > upperLimit_Address)
    {
      return;
    }
    const size_t index = address - lowLimit_Address;
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
    return measurement_data_[target_address - lowLimit_Address].current_;
  }

  float getVoltageData(const uint8_t &target_address)
  {
    return measurement_data_[target_address - lowLimit_Address].voltage_;
  }
};

std::vector<INA226> INA;
std::vector<float> rxFloatData;

std::array<INA226BiasData, INA226_MAX_NUM> ina226_all_bias_data;
std::unordered_map<uint8_t, INA226BiasData> ina226_detected_bias_data; // アドレスをハッシュとした連想配列

Ina226MeasurementData measured_data;

size_t txData_length = 0;
std::vector<uint8_t> txData(txData_length);

CRC16 CRC;
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
  for (const auto &address : readable_Addresses) // setは狭義の弱順序に従うので、順番に来てくれる
  {
    INA.emplace_back(address);

    if (INA.back().begin())
    {
      INA.back().setMaxCurrentShunt(38.73, 0.002);
      INA.back().setShuntVoltageConversionTime(4);
      INA.back().setAverage(2); // センサ値のsample数を決定する ここでのsample数は16
    }
  }
}

void setup()
{
#if DEBUG
  initializeSerial(serialBaudrate);
#endif // DEBUG
  initializeSerial1(serial1Baudrate);
  initializeSDcard();

  Wire.begin();
  Wire.setClock(400000L); //! I2C Set clock change 100kHz to 400kHz

  I2cScanner();
  pinMode(txdenPin, OUTPUT);
  setUpSDWriteTask();
}

void loop()
{

  setupINA226s();

  while (true)
  {

    measured_data.clearData();

    txData.clear();
    // stopwatch.start();
    digitalWrite(txdenPin, LOW);
    for (auto &ina_sensor : INA)
    {
      if (ina_sensor.begin())
      {
        //! is Connected
        const uint8_t target_address = ina_sensor.getAddress();
        if (ina226_detected_bias_data.find(target_address) == ina226_detected_bias_data.end()) // バイアスが設定されていない時
        {
          measured_data.setData(target_address, ina_sensor.getBusVoltage(), ina_sensor.getCurrent_mA());
        }
        else
        {
          float current_mA = ina_sensor.getCurrent_mA();
          // float voltage_V = ina_sensor.getBusVoltage();
          measured_data.setData(target_address, ina_sensor.getBusVoltage() + ina226_detected_bias_data[target_address].getVoltage(), current_mA + (ina226_detected_bias_data[target_address].getCurrent() * (current_mA / 1000.0f)));
          // Serial.printf("Getdata Address: %x, Voltage: %f, Current: %f\n", target_address, ina226_detected_bias_data[target_address].getVoltage(), ina226_detected_bias_data[target_address].getCurrent());
        }
      }
    }
    // stopwatch.printElapsedTime();
    if (Serial1.available() >= rxPacket_min_length)
    {
      uint8_t rxPacket_forward[rxPacket_forward_length] = {};
      uint8_t tx_errorStatus = 0b00000000;
      if (is_sdcard_error == true)
      {
        tx_errorStatus |= sdcard_errorStatus;
      }

      if (checkHeader(headerPacket, headerPacket_length, rxPacket_forward))
      {
        for (int i = headerPacket_length; i < rxPacket_forward_length; i++)
        {
          rxPacket_forward[i] = Serial1.read();
        }

        uint8_t rxBoardType = rxPacket_forward[headerPacket_length];
        size_t rxPacket_length = rxPacket_forward[headerPacket_length + 1];
        uint8_t rxCommand = rxPacket_forward[headerPacket_length + 2];
        uint8_t rxOption = rxPacket_forward[headerPacket_length + 3];

        //! make rxPaket
        uint8_t rxPacket[rxPacket_length] = {};
        for (int i = 0; i < rxPacket_length; i++)
        {
          if (i < rxPacket_forward_length)
          {
            rxPacket[i] = rxPacket_forward[i];
          }
          else
          {
            rxPacket[i] = Serial1.read();
          }
        }

        if (CRC.checkCrc16(rxPacket, rxPacket_length))
        {
          processCommand(rxCommand, &tx_errorStatus, rxPacket);
        }
        else
        {
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
        txPacket[packetIndex++] = (uint8_t)txPacket_length;
        txPacket[packetIndex++] = rxCommand & return_command_mask; //! command
        txPacket[packetIndex++] = tx_errorStatus;                  //! error

        //! add txData to txPacket
        if (!txData.empty())
        {
          memcpy(txPacket + packetIndex, txData.data(), txData_length);
          packetIndex += txData_length;
        }

        //! add CRC to txPacket
        uint16_t txCrc = CRC.getCrc16(txPacket, txPacket_length - crc_length);
        txPacket[packetIndex++] = lowByte(txCrc);
        txPacket[packetIndex++] = highByte(txCrc);

        // Serial.write(txPacket, txPacket_length);
        serial1SendData(txPacket, packetIndex);
      }
    } // if (Serial.available() ...

    WriteSDcard();

  } // while
} // loop

void processCommand(const uint8_t &command, uint8_t *error, const uint8_t txPacket[])
{
  txData_length = 0;
  switch (command)
  {
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

    measured_data.writeDataToBuff(txData.data());
    break;
  }
  case checkSDcardCapacityCommand:
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
  case rescanI2CCommand:
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
  case timeSetCommand:
  {
    /**
     * @brief:
     * @return:
     */
    for (int i = rxPacket_forward_length, index = 0; i < rxPacket_forward_length + JETSON_SECONDS_TIME_LENGTH; i++, index++)
    {
      seconds.uint8_tData[index] = txPacket[i];
    }
    for (int i = rxPacket_forward_length + JETSON_SECONDS_TIME_LENGTH, index = 0; i < rxPacket_forward_length + JETSON_SECONDS_TIME_LENGTH + JETSON_MILL_TIME_LENGTH; i++, index++)
    {
      milliSeconds.uint8_tData[index] = txPacket[i];
    }
    deserializeReceiveTimeData(seconds, milliSeconds);
    break;
  }
  case setupBiasCommand:
  {
    /**
     * @brief:
     * @return:
     */
    ina226_detected_bias_data.clear();
    uint8_tToFloat voltage;
    uint8_tToFloat current;
    static const size_t address_length = 1;
    static const size_t BIAS_DATA_LENGTH = address_length + (FLOAT_DATA_LENGTH + FLOAT_DATA_LENGTH);
    for (int ina_num = 0; ina_num < INA226_MAX_NUM; ina_num++)
    {
      size_t bias_head_index = rxPacket_forward_length + ina_num * BIAS_DATA_LENGTH;
      uint8_t address = txPacket[bias_head_index];
      for (int i = bias_head_index + address_length, index = 0; index < FLOAT_DATA_LENGTH; i++, index++)
      {
        voltage.uint8_tData[index] = txPacket[i];
      }
      for (int i = bias_head_index + address_length + FLOAT_DATA_LENGTH, index = 0; index < FLOAT_DATA_LENGTH; i++, index++)
      {
        current.uint8_tData[index] = txPacket[i];
      }
      ina226_all_bias_data[ina_num].setBiasData(address, voltage.floatData, current.floatData);
      // Serial.printf("Address: %x, Voltage: %f, Current: %f\n", address, ina226_all_bias_data[ina_num].getVoltage(), ina226_all_bias_data[ina_num].getCurrent());
    }

    for (const auto &address : readable_Addresses)
    {
      for (int j = 0; j < INA226_MAX_NUM; j++)
      {
        if (address == ina226_all_bias_data[j].getAddress())
        {
          ina226_detected_bias_data[address].setBiasData(address, ina226_all_bias_data[j].getVoltage(), ina226_all_bias_data[j].getCurrent());
          // Serial.printf("Update detected bias address: %x vol: %f cur: %f\n", ina226_detected_bias_data[address].getAddress(),ina226_detected_bias_data[address].getVoltage(),ina226_detected_bias_data[address].getCurrent());
          break;
        }
      }
    }
    break;
  }
  case boardResetCommand:
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
    *error |= commandUnsupport_errorStatus;
  }
}

bool checkHeader(const uint8_t header[], const size_t length, uint8_t packet[])
{
  for (int i = 0; i < length; i++)
  {
    if (Serial1.read() != header[i])
    {
      return false;
    }
    packet[i] = header[i];
  }
  return true;
}

void initializeSerial(const uint32_t &serialBaudrate)
{
  Serial.begin(serialBaudrate);
  while (!Serial)
  {
    ; //! wait for serial port to connect. Needed for native USB
  }
}

void initializeSerial1(const uint32_t &serial1Baudrate)
{
  Serial1.begin(serial1Baudrate, SERIAL_8N1, 0, 1);
  while (!Serial1)
  {
    ; //! wait for serial port to connect.
  }
}

void serial1SendData(uint8_t *txPacket, const size_t &packet_num)
{
  digitalWrite(txdenPin, HIGH);
  Serial1.write(txPacket, packet_num);
  Serial1.flush();
  digitalWrite(txdenPin, LOW);
  is_send_data = true;
}

void initializeSDcard()
{
  // SDカードのセットアップ
  Serial.println(F("Initializing SD card..."));
  if (!sd.begin(SD_CONFIG))
  {
    Serial.println(F("Failed to initialize SD card"));
    is_sdcard_error = true;
    return;
  }
  is_sdcard_error = false;

  int fileNumber = 1;   // ファイル番号の開始
  char fileName[12];    // ファイル名を格納する配列
  bool created = false; // ファイル作成フラグ

  while (!created)
  {
    sprintf(fileName, "%d.csv", fileNumber); // ファイル名を大文字で生成
    if (!sd.exists(fileName))
    {
      // ファイルが存在しない場合、新しいファイルを作成
      logData = sd.open(fileName, FILE_WRITE);
      if (logData)
      {
        Serial.print(fileName);
        Serial.println(" を作成しました。");
        created = true; // ファイルを作成したのでフラグを立てる
        logData.println("Jetson_Time, Arduino_msec, Send_Data, addr40_Voltage, addr40_Current, addr41_Voltage, addr41_Current, addr42_Voltage, addr42_Current, addr43_Voltage, addr43_Current,addr44_Voltage, addr44_Current, addr45_Voltage, addr45_Current, addr46_Voltage, addr46_Current, addr47_Voltage, addr47_Current, addr48_Voltage, addr48_Current, addr49_Voltage, addr49_Current, addr4a_Voltage, addr4a_Current, addr4b_Voltage, addr4b_Current, addr4c_Voltage, addr4c_Current, addr4d_Voltage, addr4d_Current, addr4e_Voltage, addr4e_Current, addr4f_Voltage, addr4f_Current");
      }
      else
      {
        Serial.print(fileName);
        Serial.println(" の作成に失敗しました。");
      }
    }
    fileNumber++; // 次の番号へ
  }
  while (true)
  {
    if (sd.exists(filename_for_sdcard_exists))
    {
      break;
    }
    File tmp = sd.open(filename_for_sdcard_exists, FILE_WRITE);
    if (tmp)
    {
      tmp.close();
      break;
    }
  }
  sd_writer.setSDcard(&sd, &logData);
}

void WriteSDcard()
{
  static bool is_sdcard_write = false;
  static uint32_t cached_size = 0;
  time_data = millis();
  std::string unix_time_data;
  time_manager.timeUpdate();
  // freq_calc.count();
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
    sd_writer.println(unix_time_data.c_str());
  }

  static char headerStr[32];
  memset(headerStr, 0, sizeof(headerStr));
  int32_t header_write_size = snprintf(headerStr, sizeof(headerStr), ",%d,%d,", time_data, is_send_data);
  if (header_write_size > 0)
  {
    cached_size += header_write_size;
    sd_writer.addData(headerStr, header_write_size);
  }

  is_send_data = false;

  static char dataStr[256];
  memset(dataStr, 0, sizeof(dataStr));

  int32_t wrote_size = 0;
  for (uint8_t target_address = lowLimit_Address; target_address <= upperLimit_Address; target_address++)
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
  sd_writer.addData(dataStr, wrote_size);
}

void deserializeReceiveTimeData(const uint8_tToUint32_t &seconds, const uint8_tToUint16_t &milliSeconds)
{
  time_t secondsData = seconds.uint32_tData;
  time_t milliSecondsData = milliSeconds.uint16_tData;
  time_manager.setTime(secondsData, milliSecondsData);
  // Serial.printf("Time: %s.%03d\n",buf,milliSecondsData);
}
void software_reset()
{
  ESP.restart();
}
