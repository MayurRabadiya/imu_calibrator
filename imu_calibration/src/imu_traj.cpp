#include <imu_calibration/imu_traj.h>

double angular_velocity[3];
double linear_acceleration[3];
double rx, ry, rz;

Imu_traj::Imu_traj()
{
    imuMti = nh.subscribe("GETjag/imu/data_new", 10, &Imu_traj::addImuState, this);
    odometry_sub = nh.subscribe("/GETjag/odom", 1000, &Imu_traj::odometryCallback, this);

    pubImuInitialOdometry = nh.advertise<nav_msgs::Odometry>("imuInitialState", 1000);
    pubWheelOdometry = nh.advertise<nav_msgs::Odometry>("wheelOdom", 1000);
    transform_pub_ = nh.advertise<geometry_msgs::TransformStamped>("imu_transform", 1000);
    path_pub = nh.advertise<nav_msgs::Path>("imu_path", 1000);
    imuBuffer.set_capacity(IMU_BUFFER_SIZE);
}

void Imu_traj::addImuState(const sensor_msgs::ImuConstPtr &imuMessage)
{
    ImuState imuState;
    tf::Transform transform;
    tf::Vector3 gravity(0, 0, 9.81189);
    tf::vector3MsgToTF(imuMessage->linear_acceleration, imuState.acceleration);
    imuState.timeStamp = imuMessage->header.stamp.toSec();
    imuState.orientation.setRotation(tf::Quaternion(imuMessage->orientation.x,
                                                    imuMessage->orientation.y,
                                                    imuMessage->orientation.z,
                                                    imuMessage->orientation.w));

    imuState.acceleration = imuState.acceleration - imuState.orientation.inverse() * gravity;
    
    imuState.acceleration[0] += 0.0013486125190152;
    imuState.acceleration[1] -= 0.0017497349488895;
    imuState.acceleration[2] -= 0.00092247051690049;
     
    // Time difference between two IMU messages.
    double timeDifference = imuState.timeStamp - imuBuffer.back().timeStamp;

    if (timeDifference < 0.1)
    {
        // First integration of linear_acceleration --> Velocity
        imuState.velocity = imuBuffer.back().velocity + ((imuBuffer.back().acceleration + imuState.acceleration) * timeDifference / 2.0);

        // Second Integration of linear_acceleration --> Position
        imuState.position = imuBuffer.back().position + imuState.orientation * ((imuBuffer.back().velocity + imuState.velocity) * timeDifference / 2.0);
       
    }

    imuBuffer.push_back(imuState);

    tf::Quaternion rQuat;
    imuState.orientation.getRotation(rQuat);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time().fromSec(imuState.timeStamp);

    odom_trans.header.frame_id = "odom_imu";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    imuInitialOdometry.header.stamp = ros::Time().fromSec(imuState.timeStamp);
    imuInitialOdometry.header.frame_id = "odom_imu";
    imuInitialOdometry.child_frame_id = "base_link";
    imuInitialOdometry.pose.pose.orientation = imuMessage->orientation;

    imuInitialOdometry.pose.pose.position.x = imuState.position.x();
    imuInitialOdometry.pose.pose.position.y = imuState.position.y();
    imuInitialOdometry.pose.pose.position.z = imuState.position.z();
    // imuInitialOdometry.pose.pose.position.z = 0.0;

    nav_msgs::Path path_msg;
        // Add the 3 points to the path
    geometry_msgs::PoseStamped Pose;
    path_msg.header.stamp = ros::Time().fromSec(imuState.timeStamp);
    path_msg.header.frame_id = "odom_imu";

    Pose.pose.orientation = imuMessage->orientation;
    Pose.pose.position.x = imuState.position.x();
    Pose.pose.position.y = imuState.position.y();
    Pose.pose.position.z = imuState.position.z()*0.0;
    Pose.header.frame_id = "odom_imu";
    Pose.header.stamp = ros::Time().fromSec(imuState.timeStamp);
    path_msg.poses.push_back(Pose);
    path_pub.publish(path_msg);


    pubImuInitialOdometry.publish(imuInitialOdometry);
}

void Imu_traj::odometryCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    geometry_msgs::TransformStamped wheel_odom_trans;
    wheel_odom_trans.header.stamp = odomMsg->header.stamp;

    wheel_odom_trans.header.frame_id = "odom_imu";
    wheel_odom_trans.child_frame_id = "base_link";
    wheel_odom_trans.transform.translation.x = 0;
    wheel_odom_trans.transform.translation.y = 0;
    wheel_odom_trans.transform.translation.z = 0;
    wheel_odom_trans.transform.rotation.x = 0;
    wheel_odom_trans.transform.rotation.y = 0;
    wheel_odom_trans.transform.rotation.z = 0;
    wheel_odom_trans.transform.rotation.w = 1;
    // wheel_odom_broadcaster.sendTransform(wheel_odom_trans);

    wheelOdometry.header.stamp = odomMsg->header.stamp;
    wheelOdometry.header.frame_id = "odom_imu";
    wheelOdometry.child_frame_id = "base_link_1";
    wheelOdometry.pose.pose.orientation = odomMsg->pose.pose.orientation;

    wheelOdometry.pose.pose.position = odomMsg->pose.pose.position;
    pubWheelOdometry.publish(wheelOdometry);
}