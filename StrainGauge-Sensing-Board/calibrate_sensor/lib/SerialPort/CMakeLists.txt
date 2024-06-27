cmake_minimum_required(VERSION 3.20)
project(SerialPort CXX)

set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(Boost REQUIRED COMPONENTS system thread)
option(USE_CITBRAINS_SERIALPORT_STUB "Use stub implementation of serial port" OFF)
option(CITBRAINS_SERIALPORT_STUB_DISPORSING_OUTPUT_MODE "This makes all motor output simply disposging." OFF)
add_library(serialport STATIC)
set_target_properties(serialport PROPERTIES LINKER_LANGUAGE CXX)

if(USE_CITBRAINS_SERIALPORT_STUB)
	target_sources(serialport
		PRIVATE
		SerialPortStub.h
	)
	target_compile_definitions(serialport
		PUBLIC
		USE_CITBRAINS_SERIALPORT_STUB
	)
else(USE_CITBRAINS_SERIALPORT_STUB)
	target_sources(serialport
		PRIVATE
		RealSerialPort.cpp
		RealSerialPort.h
	)
endif(USE_CITBRAINS_SERIALPORT_STUB)

if(SERIALPORT_STUB_DISPORSING_OUTPUT)
	target_compile_definitions(serialport
	PRIVATE
	ENABLE_SERVOMOTOR_STUB_DISPOSING_OUTPUT
	)
endif(SERIALPORT_STUB_DISPORSING_OUTPUT)

target_include_directories(serialport
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

target_link_libraries(serialport
	PUBLIC
	${Boost_LIBRARIES}
	pthread
)

target_compile_features(serialport
	PRIVATE
	cxx_std_20
)

target_compile_options(serialport PRIVATE -Wall -Wextra -Werror -O2 )

if(BUILD_SERIALPORT_API_TEST)
	add_executable(serialport_test
		SerialPortTest.cpp
	)
	target_link_libraries(serialport_test
		serialport
	)
	enable_ninja_output_coloring(serialport_test)
endif(BUILD_SERIALPORT_API_TEST)
