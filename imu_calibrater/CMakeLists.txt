cmake_minimum_required(VERSION 3.0.2)
project(imu_calibrater)
include(ExternalProject)

ExternalProject_Add (imu_tk
	GIT_REPOSITORY "get.upb.de:/proj/repos/forks/imu_tk.git"
	UPDATE_COMMAND ""
	INSTALL_COMMAND ""
	PREFIX "imu_tk"
)
set (imu_tk_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/imu_tk/src/imu_tk/include)
set (imu_tk_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/imu_tk/src/imu_tk-build/libimu_tk.so)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
  	roscpp
  	sensor_msgs
  	std_msgs
  	operator_interface
	imu_msg
)

# ----------------------------------------------------------------------------
# Qt and other dependencies
# ----------------------------------------------------------------------------
find_package (Eigen3 REQUIRED)
find_package (Qt5 COMPONENTS Widgets REQUIRED)

catkin_package (
    LIBRARIES imu_calibrater
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include imu_calibrater
)

add_executable(imu_read_data src/imu_read_data.cpp)
add_dependencies(imu_read_data imu_tk)
target_link_libraries(imu_read_data
	${catkin_LIBRARIES}
)

add_executable (test_imu_calib src/test_imu_calib.cpp)
add_dependencies(test_imu_calib imu_tk)
target_link_libraries (test_imu_calib PUBLIC
	${imu_tk_LIBRARIES}
	Eigen3::Eigen
)
target_include_directories(test_imu_calib PUBLIC ${imu_tk_INCLUDE_DIRS})

# To read data from arduino to ros 
add_executable(ArduinoToRos src/ArduinoToRos.cpp)
target_link_libraries(ArduinoToRos ${catkin_LIBRARIES})

# CalibrationWidget
add_widget (
	NAME			IMUCalibrationWidget
	FILES			gui/IMUCalibrationWidget.cpp
	INCLUDE_DIRS	${catkin_INCLUDE_DIRS} ${imu_tk_INCLUDE_DIRS}
	LIBS			Eigen3::Eigen Qt5::Widgets ${catkin_LIBRARIES} ${imu_tk_LIBRARIES}
	DEPENDENCIES	imu_tk

)