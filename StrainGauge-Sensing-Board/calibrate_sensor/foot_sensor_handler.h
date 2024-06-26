#ifndef _FOOT_SENSOR_HANDLER_H_
#define _FOOT_SENSOR_HANDLER_H_

#include "communicate_foot_sensor.h"
#include <string>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <chrono>


enum class TargetFoot{
  Right, 
  Left,
  Both,
  None=99
};

enum class SensorChannel
{
  Ch1, 
  Ch2, 
  Ch3, 
  Ch4, 
  All, 
  None=99
};

template <class R = int32_t>
//emplate <typename R>
struct FootSensorData
{
    using Tuple4Int32 = std::tuple<int32_t, int32_t, int32_t, int32_t>;

    FootSensorData(): sensor_data_ch1(0), sensor_data_ch2(0), sensor_data_ch3(0), sensor_data_ch4(0){}
    //FootSensorData(int32_t ch1, int32_t ch2, int32_t ch3, int32_t ch4): sensor_data_ch1(ch1), sensor_data_ch2(ch2), sensor_data_ch3(ch3), sensor_data_ch4(ch4){}
    FootSensorData(R ch1, R ch2, R ch3, R ch4): sensor_data_ch1(ch1), sensor_data_ch2(ch2), sensor_data_ch3(ch3), sensor_data_ch4(ch4){}

    R sensor_data_ch1;
    R sensor_data_ch2;
    R sensor_data_ch3;
    R sensor_data_ch4;

    /*
    int32_t sensor_data_ch1;
    int32_t sensor_data_ch2;
    int32_t sensor_data_ch3;
    int32_t sensor_data_ch4;
    */

    template <class T>
    FootSensorData<R> operator*(const  T value)
    {
        FootSensorData<R> return_value;
        return_value.sensor_data_ch1 = sensor_data_ch1 * value;
        return_value.sensor_data_ch2 = sensor_data_ch2 * value;
        return_value.sensor_data_ch3 = sensor_data_ch3 * value;
        return_value.sensor_data_ch4 = sensor_data_ch4 * value;
        return return_value;
    }

    template <class T>
    FootSensorData<R> operator/(const  T value)
    {
        FootSensorData<R> return_value;
        return_value.sensor_data_ch1 = sensor_data_ch1 / value;
        return_value.sensor_data_ch2 = sensor_data_ch2 / value;
        return_value.sensor_data_ch3 = sensor_data_ch3 / value;
        return_value.sensor_data_ch4 = sensor_data_ch4 / value;
        return return_value;
    }

    FootSensorData<R> operator+(const FootSensorData<R> &rhs)
    {
        FootSensorData<R> return_value;
        return_value.sensor_data_ch1 = sensor_data_ch1 + rhs.sensor_data_ch1;
        return_value.sensor_data_ch2 = sensor_data_ch2 + rhs.sensor_data_ch2;
        return_value.sensor_data_ch3 = sensor_data_ch3 + rhs.sensor_data_ch3;
        return_value.sensor_data_ch4 = sensor_data_ch4 + rhs.sensor_data_ch4;
        return return_value;
    }

    FootSensorData<R> &operator+=(const FootSensorData<R> &rhs)
    {
        sensor_data_ch1 += rhs.sensor_data_ch1;
        sensor_data_ch2 += rhs.sensor_data_ch2;
        sensor_data_ch3 += rhs.sensor_data_ch3;
        sensor_data_ch4 += rhs.sensor_data_ch4;
        return *this;
    }

    //Tuple4Int32 getEachSensorData(const TargetFoot &target_foot)
    std::array<R, 4> getSensorDataArray(const TargetFoot &target_foot = TargetFoot::Right) const
    {
        if(target_foot == TargetFoot::Right){
            return {sensor_data_ch1, sensor_data_ch2, sensor_data_ch3, sensor_data_ch4};
        }else{
            return {sensor_data_ch1, sensor_data_ch2, sensor_data_ch3, sensor_data_ch4};
        }
    }

    void print_all()
    {
        std::cout << "\x1b[0;0H" << std::setw(12) << sensor_data_ch1 << "\n " << std::setw(12) << sensor_data_ch2 << "\n"
        << std::setw(12) << sensor_data_ch3 << "\n" << std::setw(12) << sensor_data_ch4 << "\x1b[0J" << std::endl;
    }
};

struct FootSensorCalibrateData
{
    FootSensorCalibrateData():offset_1(0), offset_2(0), offset_3(0), offset_4(0), 
    scale_1(1), scale_2(1), scale_3(1), scale_4(1)
    {}
    FootSensorCalibrateData(const FootSensorData<int32_t> &foot_sensor_data):offset_1(0), offset_2(0), offset_3(0), offset_4(0), scale_1(1), scale_2(1), scale_3(1), scale_4(1), foot_sensor(foot_sensor_data)
    {}
    FootSensorCalibrateData(FootSensorData<int32_t> foot_sensor_data, double tmp_offset_1, double tmp_offset_2, double tmp_offset_3, double tmp_offset_4, double tmp_scale_1, double tmp_scale_2, double tmp_scale_3, double tmp_scale_4):offset_1(tmp_offset_1), offset_2(tmp_offset_2), offset_3(tmp_offset_3), offset_4(tmp_offset_4), 
    scale_1(tmp_scale_1), scale_2(tmp_scale_2), scale_3(tmp_scale_3), scale_4(tmp_scale_4), foot_sensor(foot_sensor_data)
    {}
    FootSensorCalibrateData( double tmp_offset_1, double tmp_offset_2, double tmp_offset_3, double tmp_offset_4, double tmp_scale_1, double tmp_scale_2, double tmp_scale_3, double tmp_scale_4):offset_1(tmp_offset_1), offset_2(tmp_offset_2), offset_3(tmp_offset_3), offset_4(tmp_offset_4), 
    scale_1(tmp_scale_1), scale_2(tmp_scale_2), scale_3(tmp_scale_3), scale_4(tmp_scale_4)
    {}
    FootSensorData<int32_t> foot_sensor;
    double offset_1;
    double offset_2;
    double offset_3;
    double offset_4;

    double scale_1;
    double scale_2;
    double scale_3;
    double scale_4;

    std::array<double, 4> getScale() const
    {
        return {scale_1, scale_2, scale_3, scale_4}; 
    }

    double getScale(const SensorChannel &sensor_ch)
    {
      if(sensor_ch == SensorChannel::Ch1){
          return scale_1;
      }else if(sensor_ch == SensorChannel::Ch2){
          return scale_2;
      }else if(sensor_ch == SensorChannel::Ch3){
          return scale_3;
      }else if(sensor_ch == SensorChannel::Ch4){
          return scale_4;
      }
      return -1;
    }

    std::array<double, 4> getOffset() const
    {
        return {offset_1, offset_2, offset_3, offset_4}; 
    }

    double getOffset(const SensorChannel &sensor_ch)
    {
      if(sensor_ch == SensorChannel::Ch1){
          return offset_1;
      }else if(sensor_ch == SensorChannel::Ch2){
          return offset_2;
      }else if(sensor_ch == SensorChannel::Ch3){
          return offset_3;
      }else if(sensor_ch == SensorChannel::Ch4){
          return offset_4;
      }
      return -1;
    }

    void setFootSensorData(const FootSensorData<int32_t> &foot_sensor_data)
    {

        foot_sensor = foot_sensor_data;
    }

    void setScale(const SensorChannel &sensor_ch, const double scale){
        if(sensor_ch == SensorChannel::Ch1){
            scale_1 = scale;
        }else if(sensor_ch == SensorChannel::Ch2){
            scale_2 = scale;
        }else if(sensor_ch == SensorChannel::Ch3){
            scale_3 = scale;
        }else if(sensor_ch == SensorChannel::Ch4){
            scale_4 = scale;
        }
    }

    void setOffset(const FootSensorData<int32_t> &offset_data)
    {
      offset_1 = offset_data.sensor_data_ch1;
      offset_2 = offset_data.sensor_data_ch2;
      offset_3 = offset_data.sensor_data_ch3;
      offset_4 = offset_data.sensor_data_ch4;
    }

    void setOffset(const SensorChannel &sensor_ch, const double &offset)
    {
        if(sensor_ch == SensorChannel::Ch1){
            offset_1 = offset;
        }else if(sensor_ch == SensorChannel::Ch2){
            offset_2 = offset;
        }else if(sensor_ch == SensorChannel::Ch3){
            offset_3 = offset;
        }else if(sensor_ch == SensorChannel::Ch4){
            offset_4 = offset;
        }
    }

    std::array<double, 4> getCalibrateSensorDataArray(const TargetFoot &target_foot) const
    {
        std::array<double, 4> return_calibrate_data;
        auto sensor_data_array = foot_sensor.getSensorDataArray();
        auto offset_array = getOffset();
        auto scale_array = getScale();
        for(int i=0;i<4;i++){
            return_calibrate_data.at(i) = (sensor_data_array.at(i) - offset_array.at(i)) * scale_array.at(i);
        }

        return return_calibrate_data;
    }

    void print_all(){
        std::cout.fill('0');
        std::cout << "\x1b[0;0H" << std::setw(12) << (foot_sensor.sensor_data_ch1 - offset_1) * scale_1 << "\n"
            << std::setw(12) << (foot_sensor.sensor_data_ch2 - offset_2) * scale_2 << "\n"
            << std::setw(12) << (foot_sensor.sensor_data_ch3 - offset_3) * scale_3 << "\n"
            << std::setw(12) << (foot_sensor.sensor_data_ch4 - offset_4) * scale_4 << "\n"
            << "\x1b[0J" << std::endl;
    }
};

namespace foot_sensor{
  FootSensorCalibrateData readCalibrateFile(const std::string &file_name, const TargetFoot &target_foot=TargetFoot::Right);
}//end namespace foot_sensor

class FootSensorHandler
{
public:
  FootSensorHandler() = delete;
  FootSensorHandler(const TargetFoot &target_foot = TargetFoot::None);
  ~FootSensorHandler();
  std::optional<FootSensorData<int32_t>> getOneSensorData();
  //std::pair<std::vector<int32_t>, int32_t> getTimesAverageSensorData(const size_t &);
  std::pair<std::vector<FootSensorData<int32_t>>, FootSensorData<int32_t>> getTimesAverageSensorData(const size_t &, const bool &is_display_sensor_data=false);
  FootSensorData<int32_t> decodeRawFootSensorData(const std::array<uint8_t, 32> &);
  std::optional<FootSensorData<int32_t>> RCFilter();
  //void calibrateOffset(const size_t &);
  void send();
private:
  CommunicateFootSensor communication_foot_sensor_;
  //FootSensorData foot_sensor_data_;
  TargetFoot target_foot_;
  std::string port_name_;
  std::array<uint8_t, 6> check_recv_data;
  std::size_t counter, communication_all_times;
  FootSensorData<int32_t> prev_sensor_data_;
};

#endif //
