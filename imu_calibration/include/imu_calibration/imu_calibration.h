#ifndef IMU_CALIBRATION_H_
#define IMU_CALIBRATION_H_

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

#include <imu_calibration/imu_filter.h>
#include <imu_msg/imu_custom_msg.h>

class imu_data_calibration
{
public:
	imu_data_calibration(ros::NodeHandle nh, ros::NodeHandle nhP);

private:
	ros::Publisher imuPub;

	std::vector<ros::Subscriber> subscribers;
	std::vector<double> accelBias;
	std::vector<double> gyroBias;
	Eigen::Matrix3d gyroCorrection;
	Eigen::Matrix3d accelCorrection;
	sensor_msgs::Imu msg;

	tf2_ros::TransformBroadcaster tf_broadcaster_;

	// **** paramaters
	WorldFrame::WorldFrame world_frame_;

	bool calibration;
	bool stateless_;
	bool publish_tf_;
	bool reverse_tf_;
	double constant_dt_;
	bool remove_gravity_vector_;
	double orientation_variance_;
	int no_of_imu;

	// **** state variables
	boost::mutex mutex_;
	bool initialized_;
	ros::Time last_time_;

	// **** filter implementation
	ImuFilter filter_;

	void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg);
	void imuArduino(const imu_msg::imu_custom_msg &arduinoMsg);
	void gyroCalibration();
	void accelCalibration();
	void madgwickFilter_();
	void publishFilteredMsg();
	void publishTransform();
};

#endif //IMU_CALIBRATION_H_