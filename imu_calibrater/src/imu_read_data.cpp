#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <iostream>

double angular_velocity[3];
double linear_acceleration[3];

std::fstream dataFile;
ros::Time startTime;

void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
	auto timeStamp = ros::Time::now() - startTime;

	dataFile << std::scientific << std::setprecision(10) << "   " << timeStamp
			<< "   " << imuMsg->linear_acceleration.x << "   "
			<< imuMsg->linear_acceleration.y << "   "
			<< imuMsg->linear_acceleration.z << "   "
			<< imuMsg->angular_velocity.x << "   " << imuMsg->angular_velocity.y
			<< "   " << imuMsg->angular_velocity.z << std::endl;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "imu_listener");
	ros::NodeHandle nh;
	dataFile.open("test_data/xsens_data.mat", std::ios::out);

	if (!dataFile) {
		std::cout << "File not created!";
		return -1;
	}

	startTime = ros::Time::now();
	ros::Subscriber sub = nh.subscribe("GETjag/imu/data", 1000, imuCallback);
	ros::spin();
	dataFile.close();
	return 0;
}
