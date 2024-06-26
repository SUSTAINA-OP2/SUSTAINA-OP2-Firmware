#include "foot_sensor_handler.h"
#include "foot_sensor_save_manager.h"


int main(int argc, char** argv)
{
  /* sample
     FootSensorHandler foot_sensor_handler(static_cast<int32_t>(SensorChannel::ch3));
     auto [each_data, average_value] = foot_sensor_handler.getTimesAverageSensorData(100) ;
     size_t i=0;
     std::for_each(each_data.begin(), each_data.end(), [&i](const auto &x){std::cout << x << ", " << i;i++;});
     std::cout << std::endl;
     std::cout << average_value << std::endl;
  //std::cout << foot_sensor_handler.getTimesAverageSensorData(100) << std::endl;
   */

  std::array<SensorChannel, 4> sensor_channel{SensorChannel::Ch1, SensorChannel::Ch2, SensorChannel::Ch3, SensorChannel::Ch4};
  std::vector<std::string> sensor_position{"ch1", "ch2", "ch3", "ch4"};
  std::vector<std::string> target_foot{"Right", "Left", "None"};

  // 計測するのはどっちの足なのか
  std::cout << "target_foot right or left \n right is 0, left is 1 input\nonly test is 2" << std::endl;
  TargetFoot target_foot_enum = TargetFoot::Left;
  int32_t target_foot_num=0;
  std::cin >> target_foot_num;

  if( target_foot_num < 0 || 2 < target_foot_num ){
    std::cout << "end" << std::endl;
    return -1;
  }
  
  std::string target_foot_name = target_foot.at(target_foot_num);
  if(target_foot_name == "Right"){
    target_foot_enum = TargetFoot::Right;
  }else if(target_foot_name == "Left"){
    target_foot_enum = TargetFoot::Left;
  }else{
    target_foot_enum = TargetFoot::None;
  }
  int32_t input_channel=0;
  std::cout << "input sensor 1~4 or 10 all output or end 99" << std::endl;
  std::cin >> input_channel;
  SensorChannel target_sensor;
  std::string target_position;

  bool is_output_all = false;
  //int32_t offset=0;
  //double scale=0.0;
  FootSensorCalibrateData calibrated_data;
  if(target_foot_enum != TargetFoot::None){
    calibrated_data = foot_sensor::readCalibrateFile("calibrate_test.yaml", target_foot_enum);
  }

  if(((0 < input_channel) && (input_channel <= 4)))
  {
    target_sensor = sensor_channel.at(input_channel - 1);
    target_position = sensor_position.at(input_channel-1);
    //std::cout << "input scale" << std::endl;
    //std::cin >> scale;
    //std::cout << scale << std::endl;
  }else if( input_channel == 10 ){
    is_output_all = true;

    //get offset
  }else if (input_channel == 99){
    std::cout << "end" << std::endl;
    std::exit(0);
  }else{
    std::cout << "1~4 select" << std::endl;
    return -1;
  }
  FootSensorHandler foot_sensor_handler(target_foot_enum);

  if(is_output_all == true){
    std::string str;
    int times = 0;
    std::cout << "ready calibration times or push enter(1000times get sensor)" << std::endl;
    std::cin.ignore();
    std::getline(std::cin, str);

    auto strToint = [](const std::string &str) -> int{
      try{
        int num = std::stoi(str);
        return num ;
      }catch  (std::invalid_argument) {
        return -1;
      }
    };

    if(str.empty()){
      times = 1000;
    }else{
        times = strToint(str);
        if(times == -1){
          std::cout << "invalit number set times = 1000" << std::endl;
          times = 1000;
        }
    }
    auto [ each_data_vec, average_value ] = foot_sensor_handler.getTimesAverageSensorData(times);
    calibrated_data.setOffset(average_value);
  }

  while(true)
  {
    if(is_output_all == true)
    {
      auto [each_data, average_value] = foot_sensor_handler.getTimesAverageSensorData(1) ;
      calibrated_data.setFootSensorData(average_value);
      calibrated_data.print_all();

    }else{
      double scale = 0.0;
      std::cout << "input sensor 1~4 and scale end 99" << std::endl;
      std::cin >> input_channel;
      SensorChannel target_sensor_channel = sensor_channel.at(input_channel - 1);
      std::string save_file_name = target_foot_name + sensor_position.at(input_channel-1) + "_check_result";
      scale = calibrated_data.getScale(target_sensor_channel);
      FootSensorSaveManager save_manager(save_file_name);


      int32_t weight=0, times=0;

      auto FootSensorDataToVector = [](const FootSensorData<int32_t> &sensor_data)
      {
        std::vector<int32_t> change_sensor_data(4);
        change_sensor_data.at(0) = sensor_data.sensor_data_ch1;
        change_sensor_data.at(1) = sensor_data.sensor_data_ch2;
        change_sensor_data.at(2) = sensor_data.sensor_data_ch3;
        change_sensor_data.at(3) = sensor_data.sensor_data_ch4;

        return change_sensor_data;
      };

      std::cout << "ready calibration times " << std::endl;
      std::cin >> times;
      auto [ offset_each_data_vec, offset_average_value ] = foot_sensor_handler.getTimesAverageSensorData(times);
      std::vector<int32_t> offset_each_average_value = FootSensorDataToVector(offset_average_value);
      double offset = offset_each_average_value.at(static_cast<int32_t>(target_sensor_channel));

      std::cout << "input weight and times " << std::endl;
      std::cin >> weight >> times;

      auto [each_data, average_value] = foot_sensor_handler.getTimesAverageSensorData(times, true) ;

      std::vector<int32_t> each_average_value = FootSensorDataToVector(average_value);

      std::vector<std::vector<int32_t>> all_data(4);
      for(std::size_t i = 0;i<times;i++){
        all_data.at(0).push_back(each_data.at(i).sensor_data_ch1);
        all_data.at(1).push_back(each_data.at(i).sensor_data_ch2);
        all_data.at(2).push_back(each_data.at(i).sensor_data_ch3);
        all_data.at(3).push_back(each_data.at(i).sensor_data_ch4);
      }
      std::vector<int32_t> target_sensor_data = all_data.at(static_cast<int32_t>(target_sensor));
      std::vector<int32_t> value_vec;
      for_each(target_sensor_data.cbegin(), target_sensor_data.cend(), [&value_vec, &offset, &scale](const int&sensor_data){value_vec.push_back((sensor_data - offset) * scale);});
      //int32_t value = (target_sensor_data.at(static_cast<int32_t>(target_sensor)) - offset) * scale;
      save_manager.saveScaleData(value_vec, times, weight, offset, scale);

      /*
      std::vector<int32_t> target_sensor_data(4);
      target_sensor_data.at(0) = average_value.sensor_data_ch1;
      target_sensor_data.at(1) = average_value.sensor_data_ch2;
      target_sensor_data.at(2) = average_value.sensor_data_ch3;
      target_sensor_data.at(3) = average_value.sensor_data_ch4;
      */

      //std::cout << value << std::endl;
    }
  }

  return 0;
}
