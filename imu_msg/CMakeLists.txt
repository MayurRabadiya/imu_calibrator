cmake_minimum_required(VERSION 3.0.2)

project(imu_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  imu_custom_msg.msg
)

  
generate_messages(
  DEPENDENCIES
  std_msgs 
)


catkin_package(
	CATKIN_DEPENDS
		  message_runtime
	    std_msgs

)