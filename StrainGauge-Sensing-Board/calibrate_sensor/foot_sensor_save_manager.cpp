#include "foot_sensor_save_manager.h"
#include <algorithm>


FootSensorSaveManager::FootSensorSaveManager(const std::string &target_foot): offset_file_name_("offset_file"), scale_file_name_("scale_file")
{
  //auto now = std::chrono::system_clock::now();
  
  scale_file_name_ = scale_file_name_ + "_" + target_foot + ".csv";
  offset_file_name_ = offset_file_name_ + "_" + target_foot + ".csv";
  save_offset_file_.open(offset_file_name_, std::ios::app);
  save_scale_file_.open(scale_file_name_, std::ios::app);
  if(!save_offset_file_ || !save_scale_file_){
    std::cout << "cannot open file " << std::endl;
    std::exit(EXIT_FAILURE);
  }
  /*
  scale 
    weight repeat_num, offset, value, last value is average
  offset 
    average data
  */
}

FootSensorSaveManager::~FootSensorSaveManager()
{
  std::cout << "ininininin" << std::endl;
  save_offset_file_.close();
  save_scale_file_.close();
}

void FootSensorSaveManager::saveOffsetData(const std::vector<int32_t>& data, const size_t &times, const int32_t &weight)
{
  save_offset_file_ << times;
  int64_t sum = 0;
  //std::for_each(data.begin(), data.end(), [&save_offset_file_, &sum](const auto &x){ save_offset_file_ << ", " << x; sum+=x;});
  std::for_each(data.begin(), data.end(), [this, &sum](const auto &x){ save_offset_file_ << ", " << x; sum+=x;});
  save_offset_file_ << ", " << (sum / (int64_t)times);
  save_offset_file_ << std::endl;
} 

void FootSensorSaveManager::saveScaleData(const std::vector<int32_t>& data, const size_t &times, const int32_t &weight, const int32_t &offset, const double &scale)

{
  save_scale_file_ << weight << ", " << times << ", " << offset << ", " << scale;
  int64_t sum = 0;
  std::for_each(data.begin(), data.end(), [this, &sum](const auto &x){ save_scale_file_ << ", " << x; sum+=x;});
  save_scale_file_ << ", " << (sum / (int64_t)times);
  save_scale_file_ << std::endl;

} 

void FootSensorSaveManager::saveScaleData(const std::vector<int32_t>& data, const std::vector<int32_t>& raw_data, const size_t &times, const int32_t &weight, const int32_t &offset, const double &scale)
{
  save_scale_file_ << weight << ", " << times << ", " << offset << ", " << scale;
  int64_t sum = 0, raw_sum=0;
  std::for_each(data.begin(), data.end(), [this, &sum](const auto &x){ save_scale_file_ << ", " << x; sum+=x;});
  save_scale_file_ << ", " << double (sum / (int64_t)times);
  save_scale_file_ << "\n";
  save_scale_file_ << weight << ", " << times << ", " << offset << ", " << scale;
  std::for_each(raw_data.begin(), raw_data.end(), [this, &raw_sum](const auto &x){ save_scale_file_ << ", " << x; raw_sum+=x;});
  save_scale_file_ << ", " << (raw_sum / (int64_t)times);
  save_scale_file_ << "\n" << std::endl;

} 

