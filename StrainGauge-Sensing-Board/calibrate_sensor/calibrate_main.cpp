#include "foot_sensor_handler.h"
#include "foot_sensor_save_manager.h"


int main(int argc, char** argv)
{
  /* sample
  CalibrationFootSensor foot_sensor_handler(static_cast<int32_t>(SensorChannel::ch3));
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
  std::cout << "target_foot right or left \n right is 0, left is 1 input\nonly test is 2" << std::endl;
  int32_t target_foot_num=0;
  std::cin >> target_foot_num;

  TargetFoot target_foot_enum = TargetFoot::Left;
  if(!(target_foot_num == 0 || target_foot_num == 1 || target_foot_num == 2))
  {
  	std::cout << "end" << std::endl;
    return -1;
  }
  if(target_foot.at(target_foot_num) == "Right"){
  	target_foot_enum = TargetFoot::Right;
  }else if(target_foot.at(target_foot_num) == "None"){
    target_foot_enum = TargetFoot::None;
  }
  std::cout << "" << std::endl;
  FootSensorCalibrateData calibrated_data;
  while(true)
  {
    int32_t input_channel=0;
    std::cout << "input sensor 1~4 or end 99" << std::endl;
    std::cin >> input_channel;
    SensorChannel target_sensor;
    std::string target_position = target_foot.at(target_foot_num) + "_";
    if(((0 < input_channel) && (input_channel <= 4)))
    {
      target_sensor = sensor_channel.at(input_channel - 1);
      target_position += sensor_position.at(input_channel-1);
    }else if (input_channel == 99){
      std::cout << "end" << std::endl;
      std::exit(0);
    }else{
      std::cout << "1~4 select" << std::endl;
      continue;
    }
    FootSensorHandler foot_sensor_handler(target_foot_enum);
    FootSensorSaveManager save_manager(target_position);
    int32_t weight=0, times=0;
    std::cout << "input offset get times" << std::endl;
    std::cin >> times;
    bool is_print = false;
    auto [offset_each_data, offset_average_value] = foot_sensor_handler.getTimesAverageSensorData(times, is_print) ;
    
    std::cout << "offset_average_value " << offset_average_value.sensor_data_ch1 << std::endl;
    std::cout << "offset_average_value " << offset_average_value.sensor_data_ch2 << std::endl;
    std::cout << "offset_average_value " << offset_average_value.sensor_data_ch3 << std::endl;
    std::cout << "offset_average_value " << offset_average_value.sensor_data_ch4 << std::endl;

    calibrated_data.setOffset(offset_average_value);
    std::vector<std::vector<int32_t>> offset_all_data(4);
    for(std::size_t i = 0;i<times;i++){
      offset_all_data.at(0).push_back(offset_each_data.at(i).sensor_data_ch1);
      offset_all_data.at(1).push_back(offset_each_data.at(i).sensor_data_ch2);
      offset_all_data.at(2).push_back(offset_each_data.at(i).sensor_data_ch3);
      offset_all_data.at(3).push_back(offset_each_data.at(i).sensor_data_ch4);
    }
    std::vector<int32_t> target_sensor_data_offset = offset_all_data.at(static_cast<int32_t>(target_sensor));
    int32_t offset = calibrated_data.getOffset(target_sensor);
    save_manager.saveOffsetData(target_sensor_data_offset, times, weight);

    std::cout << "input weight and times" << std::endl;
    std::cin >> weight >> times;

    auto [each_data, average_value] = foot_sensor_handler.getTimesAverageSensorData(times) ;

    //全データを入れる
    std::vector<std::vector<int32_t>> all_data(4);
    for(std::size_t i = 0;i<times;i++){
      all_data.at(0).push_back(each_data.at(i).sensor_data_ch1);
      all_data.at(1).push_back(each_data.at(i).sensor_data_ch2);
      all_data.at(2).push_back(each_data.at(i).sensor_data_ch3);
      all_data.at(3).push_back(each_data.at(i).sensor_data_ch4);
    }
    std::vector<int32_t> target_sensor_data = all_data.at(static_cast<int32_t>(target_sensor));
    if(weight == 0){
      save_manager.saveOffsetData(target_sensor_data, times, weight);
    }else{
      std::vector<int32_t> data_minus_offset;
      std::for_each(target_sensor_data.begin(), target_sensor_data.end(), 
      [&data_minus_offset,&offset](const auto &data){
        data_minus_offset.push_back(data - offset);
        });
      save_manager.saveScaleData(data_minus_offset, target_sensor_data, times, weight, offset);
    }
  }
  return 0;
}
