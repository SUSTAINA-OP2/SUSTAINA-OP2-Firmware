#ifndef _FOOT_SENSOR_SAVE_MANAGER_H_
#define _FOOT_SENSOR_SAVE_MANAGER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <filesystem>
#include <chrono>

//#include "foot_sensor_handler.h"//enumの情報だけほしい，enumを別ファイルに分離する？

enum class saveType
{
  Offset, 
  Scale,
};

enum class CalibrationMode{
  All, 
  Each
};
class FootSensorSaveManager
{
public:
  //FootSensorSaveManager(const std::string &, const CalibrationMode &mode = CalibrationMode::Each);
  FootSensorSaveManager(const std::string &);
  FootSensorSaveManager() = delete;
  ~FootSensorSaveManager();
  //template<class datetype, size_t size> 
  //void saveData(const std::array<datatype, size> & );
  void saveOffsetData(const std::vector<int32_t> &/*sensor_data*/, const size_t &/*times*/, const int32_t &/*weight*/);

  void saveScaleData(const std::vector<int32_t> &/*sensor_data*/, const size_t &/*times*/, const int32_t &/*weight*/, const int32_t &/*offset*/, const double &scale = 0.0);
  void saveScaleData(const std::vector<int32_t> &/*sensor_data*/, const std::vector<int32_t> &/*raw sensor_data*/, const size_t &/*times*/, const int32_t &/*weight*/, const int32_t &/*offset*/, const double &scale=0.0);
private:
  std::string offset_file_name_;
  std::string scale_file_name_;
  std::ofstream save_offset_file_;
  std::ofstream save_scale_file_;
  std::string time_now_;
  //const CalibrationMode calibrate_mode_;
};

#endif
