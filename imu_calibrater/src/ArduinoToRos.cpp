#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <imu_msg/imu_custom_msg.h>

double angular_velocity[3];
double linear_acceleration[3];
ros::Publisher imuPub;
sensor_msgs::Imu msg;


void gyroCallback (const imu_msg::imu_custom_msg &arduinoMsg)
{   
    msg.header.stamp = ros::Time::now();

    msg.header.frame_id= "arduino_to_ros" ;
    msg.angular_velocity.x = arduinoMsg.data[0];
    msg.angular_velocity.y = arduinoMsg.data[1];
    msg.angular_velocity.z = arduinoMsg.data[2];
    msg.linear_acceleration.x = arduinoMsg.data[3];
    msg.linear_acceleration.y = arduinoMsg.data[4];
    msg.linear_acceleration.z = arduinoMsg.data[5];


    imuPub.publish(msg);
}

int main (int argc, char *argv[])
{
    ros::init (argc, argv, "arduino_to_ros");
    ros::NodeHandle nh;
    ros::Subscriber imuArduino = nh.subscribe ("/imu/data/raw", 100, gyroCallback);
    
    imuPub = nh.advertise<sensor_msgs::Imu> ("GETjag/imu/data", 100);
    ros::spin();
    // ros::Rate r (101);
    return 0;
}