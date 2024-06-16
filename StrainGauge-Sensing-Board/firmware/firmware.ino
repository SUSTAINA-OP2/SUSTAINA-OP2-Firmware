#include <ADS126X.h>
#include <array>
#include <vector>

#include "./src/CRC16/CRC16.h"


#define NUM_AVG 50

uint8_t rx = 0;
uint8_t tx = 1;
uint8_t TXDEN = 2;

//constexpr std::array<uint8_t, 5> check_recv_data{250, 240, 245, 230, 252};
constexpr std::array<uint8_t, 8> check_recv_data{0xfe, 0xfe, 0x7D, 0x08, 0x81, 0x00, 0xf2, 0x6c};

//* check packet
constexpr uint8_t HEADER_PACKET[] = {0xfe, 0xfe};
constexpr size_t HEADER_PACKET_LENGTH = sizeof(HEADER_PACKET);
// コマンド
constexpr uint8_t READ_SENSOR_DATA_COMMAND = 0x81;
constexpr uint8_t PACKET_OPTION = 0x00;

constexpr uint8_t RETURN_PACKET_ID = 0x7D;
constexpr uint8_t CRC_LENGTH = sizeof(uint16_t);

constexpr size_t rxPacket_forward_length = HEADER_PACKET_LENGTH + 4;
constexpr size_t RECV_MIN_PACKET_SIZE = HEADER_PACKET_LENGTH + 4 + CRC_LENGTH ;

constexpr size_t txPacket_min_length = HEADER_PACKET_LENGTH + 4 + CRC_LENGTH;

constexpr size_t decode_sensor_data_length = 16;

constexpr uint8_t crc_errorStatus = 0b00000010;
constexpr uint8_t commandUnsupport_errorStatus = 0b00000010;
constexpr uint8_t commandProcessing_errorStatus = 0b00000100;

size_t txData_length = 0;
std::vector<uint8_t> txData;

CRC16 CRC;

void serial1SendData(uint8_t *txPacket, const size_t &packet_num);
bool checkHeader(const uint8_t header[], const size_t length, uint8_t packet[]);
void processCommand(const uint8_t command, uint8_t *tx_errorStatus, uint8_t rxPacket[]);

constexpr long BAUDRATE = 1000000;



bool readSerial();
void decodeSendData(const std::array<int32_t, 4> &data) ;


ADS126X adc;  // start the class
std::array<int32_t, 4> force;
//int32_t force;

int chip_select = 10;  // Arduino pin connected to CS on ADS126X

// int pos_pin = 6;  // ADS126X pin AIN0, for positive input
// //int pos_pin = ADS126X_AIN0;

// int neg_pin = 7;  // ADS126X pin AIN1, for negative input
std::array<uint8_t, 4> pos_pin{ 0, 2, 4, 6 };
std::array<uint8_t, 4> neg_pin{ 1, 3, 5, 7 };

/*--------------------- ADS ---------------------*/
void initADS() {
  //reset ads1262
  digitalWrite(10, LOW);
  delayMicroseconds(2);  // minimum of 4 t_clk = 0.54 us
  digitalWrite(10, HIGH);
  delayMicroseconds(4);  // minimum of 8 t_clk = 1.09 us

  adc.begin(chip_select);  // setup with chip select pin
  adc.setGain(ADS126X_GAIN_32);
  adc.setRate(ADS126X_RATE_38400);  //ADS126X_RATE_7200
  //adc.setRate(ADS126X_RATE_14400);
  adc.setFilter(ADS126X_SINC4);
  adc.enableInternalReference();
  adc.startADC1();  // start conversion on ADC1

  adc.disableStatus();
  adc.disableCheck();
  adc.setDelay(ADS126X_DELAY_0);
  adc.clearResetBit();
}

int32_t offset = 0;
long count = 0;
void setup() {
  Serial1.begin(BAUDRATE, SERIAL_8N1, rx, tx);

  initADS();

  delay(10);
  //int32_t count = 0;
  //double average_fps = 0;

  pinMode(10, OUTPUT);
  pinMode(TXDEN, OUTPUT);
}
double average_fps = 0;
int32_t fps_count = 0;
//std::array<int32_t, NUM_AVG> previous_values;
int current_sensor = 0;

std::array<std::array<int32_t, NUM_AVG>, 4> previous_values;

void loop() {

  //fps_count++;
  //static unsigned long start = micros();

  //adc.readADC1(pos_pin, neg_pin);

  adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);
  delayMicroseconds(250);


  // actual read
  //int32_t reading = adc.readADC1(pos_pin, neg_pin);
  int32_t reading = adc.readADC1(pos_pin[current_sensor], neg_pin[current_sensor]);
  // Serial.println(reading);

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
    //std::array<int32_t, NUM_AVG> current_previous = previous_values;
    std::array<int32_t, NUM_AVG> current_previous = previous_values[current_sensor];
    std::sort(current_previous.begin(), current_previous.end());


    // check if current reading deviates too strongly from median
    /*
    if (abs(current_previous[NUM_AVG / 2] - reading) < 1e7) {
      //force = reading;
      force[current_sensor] = reading;
    }
    */

    // update previous values for median calculation
    //previous_values[count] = reading;
    previous_values[current_sensor][count] = reading;

    std::array<int32_t, NUM_AVG> current_values = previous_values[current_sensor];
    std::sort(current_values.begin(), current_values.end());

    force[current_sensor] = current_values[NUM_AVG / 2];

    // update current sensor
    current_sensor = (current_sensor + 1) % 4;
    // Serial.println(current_sensor);
    // Serial.print("count = "); Serial.println(count);

    // update counter for median calculating
    if (!current_sensor) {
      count = (count + 1) % NUM_AVG;
    }
  }
    bool fg = readSerial();
    if(fg) {
      size_t sensor_count = 0;
      txData.resize(decode_sensor_data_length);
      txData_length = decode_sensor_data_length;
      for (const auto &sensor_in : force) {
        //Serial.print("sensor "); Serial.print(count); Serial.print(" "); Serial.println(sensor_in);
        txData[sensor_count++] = (byte)((sensor_in & 0x000000FF) >> 0);
        txData[sensor_count++] = (byte)((sensor_in & 0x0000FF00) >> 8);
        txData[sensor_count++] = (byte)((sensor_in & 0x00FF0000) >> 16);
        txData[sensor_count++] = (byte)((sensor_in & 0xFF000000) >> 24);
      }
        //! tx packet: headder + (command + length + error) + txData + crc
        //! data: (address + voldtage + cureent) * n
        size_t txPacket_length = 24;

        //! make txPacket
        uint8_t txPacket[txPacket_length] = {};
        size_t packetIndex = 0;

        //! add forward txPacket
        memcpy(txPacket, HEADER_PACKET, HEADER_PACKET_LENGTH);
        packetIndex += HEADER_PACKET_LENGTH;

        txPacket[packetIndex++] = RETURN_PACKET_ID; //! board type
        txPacket[packetIndex++] = (uint8_t)txPacket_length;
        txPacket[packetIndex++] = READ_SENSOR_DATA_COMMAND; //! command
        txPacket[packetIndex++] = PACKET_OPTION; // option
        //txPacket[packetIndex++] = tx_errorStatus; //! error

        //! add txData to txPacket
        memcpy(txPacket + packetIndex, txData.data(), txData_length);
        packetIndex += txData_length;

        //! add CRC to txPacket
        uint16_t txCrc = CRC.getCrc16(txPacket, txPacket_length - CRC_LENGTH);
        txPacket[packetIndex++] = lowByte(txCrc);
        txPacket[packetIndex++] = highByte(txCrc);

        // Serial.write(txPacket, txPacket_length);
        serial1SendData(txPacket, packetIndex);

    }
  /*
    if (Serial1.available() >= RECV_MIN_PACKET_SIZE)
    {
      uint8_t rxPacket_forward[RECV_MIN_PACKET_SIZE] = {};
      uint8_t tx_errorStatus = 0b00000000;

      if (checkHeader(HEADER_PACKET, HEADER_PACKET_LENGTH, rxPacket_forward))
      {
        for (int i = HEADER_PACKET_LENGTH; i < rxPacket_forward_length; i++)
        {
          rxPacket_forward[i] = Serial1.read();
        }

        uint8_t rxBoardType = rxPacket_forward[HEADER_PACKET_LENGTH]; // id
        size_t rxPacket_length = rxPacket_forward[HEADER_PACKET_LENGTH + 1]; // length
        uint8_t rxCommand = rxPacket_forward[HEADER_PACKET_LENGTH + 2]; // command

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
        memcpy(txPacket, HEADER_PACKET, HEADER_PACKET_LENGTH);
        packetIndex += HEADER_PACKET_LENGTH;

        txPacket[packetIndex++] = RETURN_PACKET_ID; //! board type
        txPacket[packetIndex++] = (uint8_t)txPacket_length;
        txPacket[packetIndex++] = rxCommand; //! command
        txPacket[packetIndex++] = PACKET_OPTION; // option
        //txPacket[packetIndex++] = tx_errorStatus; //! error

        //! add txData to txPacket
        if (!txData.empty())
        {
          memcpy(txPacket + packetIndex, txData.data(), txData_length);
          packetIndex += txData_length;
        }

        //! add CRC to txPacket
        uint16_t txCrc = CRC.getCrc16(txPacket, txPacket_length - CRC_LENGTH);
        txPacket[packetIndex++] = lowByte(txCrc);
        txPacket[packetIndex++] = highByte(txCrc);

        // Serial.write(txPacket, txPacket_length);
        serial1SendData(txPacket, packetIndex);
      }
    } // if (Serial.available() ...
    */
  }
  //double =  (( end - start ) / 1000000.);
  //average_fps += tmp_fps ;

  /* calc fps
  if (fps_count == 1000) {
    unsigned long end = micros();
    double count_sum_times = (end - start) / 1000000.;
    //start = end;
    Serial.println(fps_count / count_sum_times);
    for (int i = 0; i < 4; i++) {
      Serial.print("sensro = ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(force[i]);
    }
    fps_count = 0;
    //average_fps = 0;
    start = micros();
  }
  */
  //delay(100);  // wait 1 second
void decodeSendData(const std::array<int32_t, 4> &data) 
{
  size_t tx_data_num = 19;
  uint8_t txCmd[tx_data_num];
  uint8_t rxCmd[9];
  txCmd[0] = 0x0f;
  txCmd[1] = 0x20;
  int sensor_count = 2;

  for (const auto &sensor_in : data) {
    //Serial.print("sensor "); Serial.print(count); Serial.print(" "); Serial.println(sensor_in);
    txCmd[sensor_count] = (byte)((sensor_in & 0x000000FF) >> 0);
    txCmd[sensor_count + 1] = (byte)((sensor_in & 0x0000FF00) >> 8);
    txCmd[sensor_count + 2] = (byte)((sensor_in & 0x00FF0000) >> 16);
    txCmd[sensor_count + 3] = (byte)((sensor_in & 0xFF000000) >> 24);
    sensor_count += 4;
  }

  uint8_t check_sum = 0;
  for (int i = 0; i < tx_data_num-1; i++)
  {
    //Serial.print(txCmd[i]);Serial.print(",");
    check_sum ^= txCmd[i];
  }
  txCmd[tx_data_num-1] = check_sum;

  digitalWrite(TXDEN, HIGH);
  Serial1.write(txCmd, sizeof(txCmd));
  delayMicroseconds(1000);
  //digitalWrite(TXDEN, LOW);

	// send data
}
bool readSerial(){
  digitalWrite(TXDEN, LOW);
  //Serial.println("readSeiral inininin");
  uint8_t incomingByte = 0;  // for incoming serial data
  std::array<uint8_t, 8> recv_data;
  bool fg = false;
  while (Serial1.available() ) {
    // read the incoming byte:
    incomingByte = Serial1.read();
    //Serial.println(incomingByte);
    int8_t correct_count = 0;
    if(check_recv_data[0] == incomingByte) {
      
      correct_count++;
      for(int i = 1; i < 8; i++) {
        incomingByte = Serial1.read();
        //Serial.println(incomingByte);
        if(check_recv_data[i] == incomingByte) {
          correct_count++;
        }else{
          break;
        }
      }
    }
    if(correct_count == 8) {
      fg = true;
      //	break;
    }
  }
  return fg;
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
void processCommand(const uint8_t command, uint8_t *tx_errorStatus, uint8_t rxPacket[])
{
  txData_length = 0;
  txData.clear();
  switch (command)
  {
  case READ_SENSOR_DATA_COMMAND:
  {
    //Serial.println("READ_SENSOR_DATA_COMMAND");
    //Serial.println("send");
    size_t sensor_count = 0;
    txData.resize(decode_sensor_data_length);
    txData_length = decode_sensor_data_length;
    for (const auto &sensor_in : force) {
      //Serial.print("sensor "); Serial.print(count); Serial.print(" "); Serial.println(sensor_in);
      txData[sensor_count++] = (byte)((sensor_in & 0x000000FF) >> 0);
      txData[sensor_count++] = (byte)((sensor_in & 0x0000FF00) >> 8);
      txData[sensor_count++] = (byte)((sensor_in & 0x00FF0000) >> 16);
      txData[sensor_count++] = (byte)((sensor_in & 0xFF000000) >> 24);
    }
    break;
  }
  default:
    *tx_errorStatus |= commandUnsupport_errorStatus;
    break;
  }
}

void serial1SendData(uint8_t *txPacket, const size_t &packet_num)
{
  digitalWrite(TXDEN, HIGH);
  Serial1.write(txPacket, packet_num);
  Serial1.flush();
  digitalWrite(TXDEN, LOW);
}
