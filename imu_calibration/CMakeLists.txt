cmake_minimum_required(VERSION 3.0.2)
project(imu_calibration)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
sensor_msgs
tf
tf2
dynamic_reconfigure
imu_filter_madgwick
message_generation
imu_msg
)

find_package (Eigen3 REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(
)

add_message_files(
  FILES
  imu_custom_msg.msg
)
  
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  LIBRARIES imu_calibration
)


include_directories(
	${catkin_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
	include imu_calibration
)

add_executable (imu_calibration src/imu_calibration_node.cpp src/imu_calibration.cpp)
add_dependencies(imu_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (imu_calibration ${catkin_LIBRARIES})
target_link_libraries(imu_calibration Eigen3::Eigen)

# To read data from arduino to ros 
add_executable(ArduinoToRos src/ArduinoToRos.cpp)
target_link_libraries(ArduinoToRos ${catkin_LIBRARIES})

add_executable (imu_traj_node src/imu_traj_node.cpp src/imu_traj.cpp)
add_dependencies(imu_traj_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (imu_traj_node ${catkin_LIBRARIES})
target_link_libraries(imu_traj_node Eigen3::Eigen)


add_library (imu_filter src/imu_filter.cpp  src/imu_filter_ros.cpp src/stateless_orientation.cpp)
add_dependencies(imu_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
