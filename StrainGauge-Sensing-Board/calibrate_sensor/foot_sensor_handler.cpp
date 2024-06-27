#include "foot_sensor_handler.h"
#include <chrono>

FootSensorHandler::FootSensorHandler(const TargetFoot &target_foot): target_foot_(target_foot), counter(0), communication_all_times(0)
{
  if(target_foot_ == TargetFoot::Right)
  {

#ifdef USE_PMX_SERVO_MOTOR
    port_name_ = "/dev/PMXforID_03_01_02+RightFootSensor";
#else
    port_name_ = "/dev/B3M_Port_1_to_5";
#endif

  }else if (target_foot_ == TargetFoot::Left){

#ifdef USE_PMX_SERVO_MOTOR
    port_name_ = "/dev/PMXforID_13_11_12+LeftFootSensor";
#else
    port_name_ = "/dev/B3M_Port_10_to_14";
#endif

  }else{

#ifdef USE_PMX_SERVO_MOTOR
    port_name_ = "/dev/ttyUSB1";
#else
    port_name_ = "/dev/ttyUSB0";
#endif
  }

  communication_foot_sensor_.init(port_name_);
}

FootSensorHandler::~FootSensorHandler()
{
}

void FootSensorHandler::send()
{
	communication_foot_sensor_.getRawData();
}

std::optional<FootSensorData<int32_t>> FootSensorHandler::getOneSensorData()
{

  std::optional<std::array<uint8_t, 32>> sensor_raw_data = communication_foot_sensor_.getRawData();
  if(!sensor_raw_data)
  {
    std::cout << "snesor data no arraival" << std::endl;
    return std::nullopt;
  }

  auto sensor_data = sensor_raw_data.value();

  if(!communication_foot_sensor_.calcCheckSum(sensor_data))
  {
    std::cout << "check sum is not match" << std::endl;
    return std::nullopt;
  }

  using namespace std::chrono_literals;
  if(!((sensor_data.at(0) == check_recv_data.at(0))&& (sensor_data.at(1) == check_recv_data.at(1))))
  //for(size_t i=0;i<check_recv_data.size();i++)
  {
    //if(sensor_data.at(i) != check_recv_data.at(i))
    {
      std::cout << "check_recv_data is not match" << std::endl;
      return std::nullopt;
    }
  }
  std::array<int32_t, 4> each_data;

  /*
  for(const auto &data : sensor_data)
  {
    std::cout << std::dec << static_cast<int32_t>(data) << " ";
    std::cout << std::hex << static_cast<int>(data) << " ";
  }
  std::cout << std::endl;
  */

   return decodeRawFootSensorData(sensor_data);
}

std::pair<std::vector<FootSensorData<int32_t>>, FootSensorData<int32_t>> FootSensorHandler::getTimesAverageSensorData(const size_t &times, const bool &is_display_sensor_data)
{
  using namespace std::chrono_literals;
  //int32_t sum_value = 0;
  //std::vector<int32_t> each_data;
  //std::vector<int32_t> sum_value_vec (4, 0);
  std::vector<FootSensorData<int32_t>> return_data;
  FootSensorData<int32_t> average_data;
  std::cout << "times = " << times << std::endl;
  //std::array<std::array<int64_t, times>, 4> sum_value_vec = {0};
  std::array<int64_t, 4> sum_value_vec = {0};
  for(size_t i=0;i<times;)
  {
    //std::vector<int32_t> tmp_each_data;
    auto tmp_all_data  = getOneSensorData();
    if(tmp_all_data)
    {
      FootSensorData all_data = tmp_all_data.value();
      return_data.push_back(all_data);
      sum_value_vec.at(0) += all_data.sensor_data_ch1;
      sum_value_vec.at(1) += all_data.sensor_data_ch2;
      sum_value_vec.at(2) += all_data.sensor_data_ch3;
      sum_value_vec.at(3) += all_data.sensor_data_ch4;

      //foot_data_sum_value += all_data;

      /*
      std::cout << "times = " << i << std::endl;
      std::cout << foot_data_sum_value.sensor_data_ch1 << std::endl;
      std::cout << foot_data_sum_value.sensor_data_ch2 << std::endl;
      std::cout << foot_data_sum_value.sensor_data_ch3 << std::endl;
      std::cout << foot_data_sum_value.sensor_data_ch4 << std::endl;
      */

      i++;
      if(is_display_sensor_data){
        all_data.print_all();
      }
      if(i % 10 == 0)
      {
        std::cout << std::endl;
        std::cout << i << " / " << times << std::endl;
      }
    }else{
      counter++;
    }
    communication_all_times++;
    std::this_thread::sleep_for(1ms);
  }
  std::cout << "get times is " << times << ", communication all times is " << counter << " / " << communication_all_times << std::endl;
  counter=0, communication_all_times=0;
  //average_data.sensor_data_ch1 = std::accumulate(sum_value_vec.at(0).begin(), sum_value_vec.at(0).end(), 0) / times;

  for(auto &sum_value : sum_value_vec)
  {
    std::cout << sum_value << std::endl;
  }

  std::array<int64_t, 4> average_value_int64;


  std::transform(sum_value_vec.begin(), sum_value_vec.end(), average_value_int64.begin(), [times](const int64_t &sum_value){return sum_value / (int64_t)times;});

  for(auto &sum_value : average_value_int64)
  {
    std::cout << sum_value << std::endl;
  }
  int64_t max_value = *std::max_element(average_value_int64.begin(), average_value_int64.end());
  int64_t min_value = *std::min_element(average_value_int64.begin(), average_value_int64.end());

    //std::cout << "max value is " << max_value << std::endl;
    //std::cout << "min value is " << min_value << std::endl;
    //std::this_thread::sleep_for(5s);

  if(max_value < std::numeric_limits<int32_t>::max() && min_value > std::numeric_limits<int32_t>::min())
  {
    average_data.sensor_data_ch1 = average_value_int64.at(0);
    average_data.sensor_data_ch2 = average_value_int64.at(1);
    average_data.sensor_data_ch3 = average_value_int64.at(2);
    average_data.sensor_data_ch4 = average_value_int64.at(3);
  }else{
    std::cout << "over flow" << std::endl;
    return {return_data, FootSensorData<int32_t>()};
  }

  //return {return_data, (foot_data_sum_value / (int32_t)times)};
  return {return_data, average_data};
}

FootSensorData<int32_t> FootSensorHandler::decodeRawFootSensorData(const std::array<uint8_t, 32> &data)
{
    FootSensorData<int32_t> foot_data;

    std::size_t count = 6;

    foot_data.sensor_data_ch1 = ((data[count+3] << 24) | (data[count+2] << 16) | (data[count+1] << 8) | data[count]);
    count+=4;
    foot_data.sensor_data_ch2 = ((data[count+3] << 24) | (data[count+2] << 16) | (data[count+1] << 8) | data[count]);
    count+=4;
    foot_data.sensor_data_ch3 = ((data[count+3] << 24) | (data[count+2] << 16) | (data[count+1] << 8) | data[count]);
    count+=4;
    foot_data.sensor_data_ch4 = ((data[count+3] << 24) | (data[count+2] << 16) | (data[count+1] << 8) | data[count]);

    return foot_data;

}

std::optional<FootSensorData<int32_t>> FootSensorHandler::RCFilter()
{
    auto tmp_sensor_data = getOneSensorData();
    FootSensorData<int32_t>  sensor_data, after_sensor_data;
    double a = 0.5;
    if(tmp_sensor_data){
        sensor_data = tmp_sensor_data.value();

        after_sensor_data = prev_sensor_data_ * a + sensor_data * (1-a);
        prev_sensor_data_ = after_sensor_data;
    }else{
        return std::nullopt;
    }

    return sensor_data;
}

namespace foot_sensor{
  FootSensorCalibrateData readCalibrateFile(const std::string &file_name, const TargetFoot &target_foot)
{
      YAML::Node node = YAML::LoadFile(file_name);
      std::string target_foot_name = "right_foot";
      YAML::Node foot_node;
      if(target_foot == TargetFoot::Left){
        target_foot_name = "left_foot";
      }

      foot_node = node[target_foot_name];
      //YAML::Node right_foot = node["right_foot"];
      //YAML::Node left_foot = node["left_foot"];

      FootSensorCalibrateData calibrate_data;
      calibrate_data.setOffset(SensorChannel::Ch1, foot_node["ch1"]["offset"].as<double>());
      calibrate_data.setOffset(SensorChannel::Ch2, foot_node["ch2"]["offset"].as<double>());
      calibrate_data.setOffset(SensorChannel::Ch3, foot_node["ch3"]["offset"].as<double>());
      calibrate_data.setOffset(SensorChannel::Ch4, foot_node["ch4"]["offset"].as<double>());

      calibrate_data.setScale(SensorChannel::Ch1, foot_node["ch1"]["scale"].as<double>());
      calibrate_data.setScale(SensorChannel::Ch2, foot_node["ch2"]["scale"].as<double>());
      calibrate_data.setScale(SensorChannel::Ch3, foot_node["ch3"]["scale"].as<double>());
      calibrate_data.setScale(SensorChannel::Ch4, foot_node["ch4"]["scale"].as<double>());

      calibrate_data.print_all();

      return calibrate_data;
  }
}//end namespace
