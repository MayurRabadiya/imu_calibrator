#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

struct ImuState
{
	tf::Vector3 position;
	tf::Vector3 velocity;
	tf::Vector3 gravity;
	tf::Vector3 angular_velocity;
	tf::Vector3 acceleration;
	tf::Matrix3x3 orientation;

	double timeStamp;
	ImuState()
	{
		timeStamp = 0.0;
		gravity = tf::Vector3(0, 0, 0);
		position = tf::Vector3(0, 0, 0);
		velocity = tf::Vector3(0, 0, 0);
		acceleration = tf::Vector3(0, 0, 0);
		angular_velocity = tf::Vector3(0, 0, 0);
		orientation = tf::Matrix3x3(tf::Quaternion(0, 0, 0, 1));
	}
};

class Imu_traj
{
public:
	Imu_traj();
	ros::NodeHandle nh;
	ros::Subscriber imuMti;
	ros::Subscriber odometry_sub;


	void addImuState(const sensor_msgs::ImuConstPtr &imuMessage);
	void odometryCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

private:
	ros::Publisher pubImuInitialOdometry;
	ros::Publisher pubWheelOdometry;
	nav_msgs::Odometry imuInitialOdometry;
	nav_msgs::Odometry wheelOdometry;
	ros::Publisher transform_pub_;
	ros::Publisher path_pub;
	tf::TransformBroadcaster odom_broadcaster;
	tf::TransformBroadcaster wheel_odom_broadcaster;
	const int IMU_BUFFER_SIZE = 100;
	boost::circular_buffer<ImuState> imuBuffer;
};
