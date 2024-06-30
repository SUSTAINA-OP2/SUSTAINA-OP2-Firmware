#ifndef COMMUNICATE_FOOT_SENSOR_H_
#define COMMUNICATE_FOOT_SENSOR_H_

//#include <KSerialPort.h>
#include "SerialPort.h"
#include <vector>
#include <array>
#include <iostream>
#include <string>
#include <optional>


class CommunicateFootSensor
{
public:
  CommunicateFootSensor();
  ~CommunicateFootSensor();
  void init(const std::string &port_name="/dev/ttyUSB0");
  bool calcCheckSum(const std::array<uint8_t, 32> &);//xorの確認
  std::optional<std::array<uint8_t, 32>> getRawData();

private:
  citbrains::SerialPort rs485_port_;
  size_t readlen;
  std::vector<uint8_t> send_buf_;

};

#endif //COMMUNICATE_FOOT_SENSOR_H_
