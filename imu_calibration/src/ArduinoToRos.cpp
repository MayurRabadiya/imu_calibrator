#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

double angular_velocity[3];
double linear_acceleration[3];
ros::Publisher imuPub;
sensor_msgs::Imu msg;
void gyroCallback (const geometry_msgs::Vector3::ConstPtr& gyroMsg)
{   
    msg.header.stamp = ros::Time::now();
    msg.angular_velocity.x = gyroMsg->x;
    msg.angular_velocity.y = gyroMsg->y;
    msg.angular_velocity.z = gyroMsg->z;
}

void accelCallback (const geometry_msgs::Vector3::ConstPtr& accelMsg)
{   
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id= "arduino_to_ros" ;
    msg.linear_acceleration.x = accelMsg->x;
    msg.linear_acceleration.y = accelMsg->y;
    msg.linear_acceleration.z = accelMsg->z;
    imuPub.publish(msg);
}

int main (int argc, char *argv[])
{
    ros::init (argc, argv, "arduino_to_ros");
    ros::NodeHandle nh;
    ros::Subscriber imuGyro = nh.subscribe ("imu_gyro", 100, gyroCallback);
    ros::Subscriber imuAccel = nh.subscribe ("imu_accl", 100,accelCallback);
    imuPub = nh.advertise<sensor_msgs::Imu> ("GETjag/imu/raw_data", 100);
    ros::spin();
    // ros::Rate r (101);
    return 0;
}