#include "communicate_foot_sensor.h"
#include <sys/resource.h>
#include <thread>
#include <chrono>

CommunicateFootSensor::CommunicateFootSensor(): readlen(24)//: send_buf_{250, 240, 245, 230, 252}
{
  //send_buf_.fill(0);



  send_buf_  = {0xfe, 0xfe, 0xA0, 0xA0, 0x81, 0x00};
  uint16_t crc_num  = crc.getCrc16(send_buf_.data(), send_buf_.size());
  send_buf_.push_back(crc_num & 0xff);
  send_buf_.push_back((crc_num >> 8) & 0xff);
}

CommunicateFootSensor::~CommunicateFootSensor()
{
}

void CommunicateFootSensor::init(const std::string &port_name)
{
  using serial = citbrains::SerialPort;
  auto pid = getpid();
  auto err_priority = setpriority(PRIO_PROCESS, pid, -20);
  if (err_priority == -1)
  {
      std::cout << strerror(errno) << std::endl;
  }
  auto err = rs485_port_.open(port_name);
  if(err)
  {
    std::cerr << "cannot open" << std::endl;
    std::cerr  << err.message() << std::endl;
    std::exit(EXIT_FAILURE);
  }
  rs485_port_.setBaudRate(1'000'000); //3M
  //rs485_port_.setBaudRate(115'200); //3M
  rs485_port_.setStopbits(serial::Stopbits::One); //2bit ストップビットは2bit
  rs485_port_.setParity(serial::Parity::None);
}

std::optional<std::array<uint8_t, 32>> CommunicateFootSensor::getRawData()
{
  using namespace std::chrono_literals;
  std::array<uint8_t, 32> recvbuf;
  recvbuf.fill(0);

  boost::system::error_code err;
  size_t  writelen = 8;
  size_t  ret=0;

  //ret = rs485_port_.write_some(reinterpret_cast<char*>(send_buf_.data()),writelen,err);
  ret = rs485_port_.writeSome(send_buf_, err);
  if(err){
    std::cerr << err.message() << std::endl;
    return std::nullopt;
  }
  std::this_thread::sleep_for(200us);
  //ret = rs485_port_.read(reinterpret_cast<char*>(recvbuf.data()),readlen,err,10000,50000);
  ret = rs485_port_.read(recvbuf.data(),readlen,err,10000,50000);

  if(err){
    std::cerr << err.message() << std::endl;
    return std::nullopt;
  }

  return recvbuf;
}

bool CommunicateFootSensor::calcCheckSum(std::array<uint8_t, 32> data)
{
    const uint_fast16_t checksum = crc.getCrc16(data.data(), readlen - 2); // crc16の計算
    const uint_fast16_t packet_checksum = static_cast<uint_fast16_t>(data[readlen - 2] |
                                                                        data[readlen - 1] << 8); // パケットの最後の2byteがcrc16
    if(checksum != packet_checksum)
    {
        std::cerr << "checksum error" << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}
