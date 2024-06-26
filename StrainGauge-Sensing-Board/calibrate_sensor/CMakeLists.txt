cmake_minimum_required(VERSION 3.10)

project(calibrate_foot_sensor CXX)
#include(../../cmake_modules/useful_setting.cmake)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/SerialPort ./SerialPort)
#add_subdirectory(SerialPort)

add_library(communicate_foot_sensor)
add_library(save_manager)

list(APPEND compile_targets calibrate_foot_sensor check_result_sensor output_data calibrate_foot_sensor_gui)

add_executable(calibrate_foot_sensor)
add_executable(check_result_sensor)
add_executable(output_data)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(calibrate_foot_sensor_gui)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(yaml-cpp 0.7.0 REQUIRED)
find_package(Eigen3 REQUIRED)

option(USE_PMX_SERVO "USE PMX Servo motor" ON)
if(USE_PMX_SERVO)
  target_compile_definitions(communicate_foot_sensor PRIVATE -DUSE_PMX_SERVO_MOTOR)
endif(USE_PMX_SERVO)

target_sources(communicate_foot_sensor PRIVATE
  communicate_foot_sensor.cpp
  communicate_foot_sensor.h
  foot_sensor_handler.cpp
  foot_sensor_handler.h
  foot_sensor_save_manager.h
  foot_sensor_save_manager.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/crc16.hpp
)

target_sources(save_manager PRIVATE
      foot_sensor_save_manager.cpp
      foot_sensor_save_manager.h
)

target_sources(calibrate_foot_sensor PRIVATE 
      calibrate_main.cpp
)
target_sources(check_result_sensor PRIVATE 
      check_calibrate_result.cpp
)

target_sources(output_data PRIVATE
      output.cpp
)

target_sources(calibrate_foot_sensor_gui PRIVATE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(communicate_foot_sensor PRIVATE ${BOOST_LIBRARIES} pthread
 yaml-cpp serialport)
target_include_directories(communicate_foot_sensor PUBLIC
  ${BOOST_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/lib/SerialPort/
  ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/lib )
target_compile_features(communicate_foot_sensor PRIVATE cxx_std_20)
target_compile_options(communicate_foot_sensor PRIVATE -O2 -march=native )

target_include_directories(save_manager PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

foreach(target ${compile_targets})
    target_link_libraries(${target} PRIVATE communicate_foot_sensor)
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_features(${target} PRIVATE cxx_std_20)
    target_compile_options(${target} PRIVATE -O2 -march=native)
    #enable_ninja_output_coloring(${target})
endforeach(target ${compile_targets})

foreach(target calibrate_foot_sensor check_result_sensor)
    target_link_libraries(${target} PRIVATE save_manager)
endforeach(target)

target_link_libraries(calibrate_foot_sensor_gui PRIVATE Qt5::Widgets Eigen3::Eigen)
target_compile_features(calibrate_foot_sensor_gui PRIVATE cxx_std_20)
