#include "foot_sensor_handler.h"
#include <iostream>


int main(int argc, char** argv)
{
    using namespace std::chrono_literals;
    //FootSensorHandler foot_sensor_handler();
    FootSensorHandler right_foot(TargetFoot::Right);
    FootSensorHandler left_foot(TargetFoot::Left);
    
    FootSensorCalibrateData right_calibrated_data = foot_sensor::readCalibrateFile("calibrate_test.yaml", TargetFoot::Right);
    FootSensorCalibrateData left_calibrated_data = foot_sensor::readCalibrateFile("calibrate_test.yaml", TargetFoot::Left);
    right_calibrated_data.print_all();
    left_calibrated_data.print_all();

    int32_t right_count=0, left_count=0;
    bool prev_in_calibrate = false;
    while(true){
      auto right_sensor_data = right_foot.getOneSensorData();
      auto left_sensor_data = left_foot.getOneSensorData();
      if(right_sensor_data){
        right_calibrated_data.setFootSensorData(right_sensor_data.value());
        std::cout << "right foot ";
        right_calibrated_data.print_all();
        right_count++;
      }
      if(left_sensor_data){
        left_calibrated_data.setFootSensorData(left_sensor_data.value());
        std::cout << "left foot ";
        left_calibrated_data.print_all();
        left_count++;
      }
      if(right_count > 500 || left_count > 500){
        std::cout << "chagne state" << std::endl;
        if(prev_in_calibrate == true){
          std::this_thread::sleep_for(1s);
          prev_in_calibrate = false;

        }else{
          std::cout << "calibrate in" << std::endl;
          std::this_thread::sleep_for(1s);
          int times = 100;
          auto [r_each_data_vec, r_average_value] = right_foot.getTimesAverageSensorData(times);
          auto [l_each_data_vec, l_average_value] = left_foot.getTimesAverageSensorData(times);
          right_calibrated_data.setOffset(r_average_value);
          left_calibrated_data.setOffset(l_average_value);
          prev_in_calibrate = true;
          std::cout << "end calibrate in" << std::endl;
          std::this_thread::sleep_for(500ms);
        }
        left_count = 0;
        right_count = 0;
      }
      std::this_thread::sleep_for(12.5ms);
    }

    return 0;
}
