#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <imu_calibration/imu_calibration.h>
#include <imu_calibration/stateless_orientation.h>

imu_data_calibration::imu_data_calibration(ros::NodeHandle nh, ros::NodeHandle nhP)
{
    std::vector<double> accelMisalignment;
    std::vector<double> accelScale;
    std::vector<double> gyroMisalignment;
    std::vector<double> gyroScale;
    bool xsens_mti;
    bool lsm6dox;
    
    imuPub = nh.advertise<sensor_msgs::Imu>("GETjag/imu/data_new", 10);

    nhP.getParam("accel_misalignment", accelMisalignment);
    nhP.getParam("accel_scale", accelScale);
    nhP.getParam("accel_bias", accelBias);
    nhP.getParam("gyro_misalignment", gyroMisalignment);
    nhP.getParam("gyro_scale", gyroScale);
    nhP.getParam("gyro_bias", gyroBias);

    // keep true for calibration and false to publish raw data.
    nhP.getParam("calibration", calibration);

    // Make true according to type of IMU used.
    nhP.getParam("xsens_mti", xsens_mti);
    nhP.getParam("lsm6dox", lsm6dox);

    // Publish IMU tf with orientation from madgwick filter.
    if (!nhP.getParam("publish_tf", publish_tf_))
        publish_tf_ = true;

    if (!nhP.getParam("reverse_tf", reverse_tf_))
        reverse_tf_ = false;

    if (!nhP.getParam("constant_dt", constant_dt_))
        constant_dt_ = 0.0;

    if (!nhP.getParam("remove_gravity_vector", remove_gravity_vector_))
        remove_gravity_vector_ = false;

    std::string world_frame;
    if (!nhP.getParam("world_frame", world_frame))
        world_frame = "enu";

    if (world_frame == "ned")
    {
        world_frame_ = WorldFrame::NED;
    }
    else if (world_frame == "nwu")
    {
        world_frame_ = WorldFrame::NWU;
    }
    else if (world_frame == "enu")
    {
        world_frame_ = WorldFrame::ENU;
    }
    else
    {
        ROS_ERROR("The parameter world_frame was set to invalid value '%s'.", world_frame.c_str());
        ROS_ERROR("Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
        world_frame_ = WorldFrame::ENU;
    }
    filter_.setWorldFrame(world_frame_);

    // check for illegal constant_dt values
    if (constant_dt_ < 0.0)
    {
        ROS_FATAL("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
        constant_dt_ = 0.0;
    }
    if (constant_dt_ == 0.0)
        ROS_INFO("Using dt computed from message headers");
    else
        ROS_INFO("Using constant dt of %f sec", constant_dt_);

    filter_.setAlgorithmGain(0.1);

    Eigen::Matrix3d gyroMisalignmentMatrix;
    Eigen::Matrix3d gyroScaleMatrix;
    gyroMisalignmentMatrix << gyroMisalignment[0], gyroMisalignment[1], gyroMisalignment[2],
        gyroMisalignment[3], gyroMisalignment[4], gyroMisalignment[5],
        gyroMisalignment[6], gyroMisalignment[7], gyroMisalignment[8];
    gyroScaleMatrix << gyroScale[0], gyroScale[1], gyroScale[2],
        gyroScale[3], gyroScale[4], gyroScale[5],
        gyroScale[6], gyroScale[7], gyroScale[8];
    gyroCorrection = gyroMisalignmentMatrix * gyroScaleMatrix;

    Eigen::Matrix3d accelMisalignmentMatrix;
    Eigen::Matrix3d accelScaleMatrix;
    accelMisalignmentMatrix << accelMisalignment[0], accelMisalignment[1], accelMisalignment[2],
        accelMisalignment[3], accelMisalignment[4], accelMisalignment[5],
        accelMisalignment[6], accelMisalignment[7], accelMisalignment[8];
    accelScaleMatrix << accelScale[0], accelScale[1], accelScale[2],
        accelScale[3], accelScale[4], accelScale[5],
        accelScale[6], accelScale[7], accelScale[8];
    accelCorrection = accelScaleMatrix * accelMisalignmentMatrix;

    if (xsens_mti)
    {
        subscribers.push_back(nh.subscribe("/GETjag/imu/data", 100, &imu_data_calibration::imuCallback, this));
    }
    if (lsm6dox)
    {
        subscribers.push_back(nh.subscribe("/imu/data/raw", 100, &imu_data_calibration::imuArduino, this));
    }
}


void imu_data_calibration::imuArduino(const imu_msg::imu_custom_msg &arduinoMsg)
{
    msg.angular_velocity.x = arduinoMsg.data[0];
    msg.angular_velocity.y = arduinoMsg.data[1];
    msg.angular_velocity.z = arduinoMsg.data[2];

    msg.linear_acceleration.x = arduinoMsg.data[3];
    msg.linear_acceleration.y = arduinoMsg.data[4];
    msg.linear_acceleration.z = arduinoMsg.data[5];

    if (calibration)
    {
        gyroCalibration();
        accelCalibration();
    }
    madgwickFilter_();
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "GETjag/imu_link";
    imuPub.publish(msg);
}

void imu_data_calibration::imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    msg.linear_acceleration.x = imuMsg->linear_acceleration.x;
    msg.linear_acceleration.y = imuMsg->linear_acceleration.y;
    msg.linear_acceleration.z = imuMsg->linear_acceleration.z;

    msg.angular_velocity.x = imuMsg->angular_velocity.x;
    msg.angular_velocity.y = imuMsg->angular_velocity.y;
    msg.angular_velocity.z = imuMsg->angular_velocity.z;

    if (calibration)
    {
        gyroCalibration();
        accelCalibration();
    }
    madgwickFilter_();
    msg.header.stamp = imuMsg->header.stamp;
    msg.header.frame_id = imuMsg->header.frame_id;
    imuPub.publish(msg);
}

/**
 * @brief Sensor model for gyroscope and accelerometer data.
 *
 * Calibrated_data = misallignment * scale_factor (sensor_data -  bias)
 */
void imu_data_calibration::gyroCalibration()
{
    Eigen::Vector3d gyroBiasVector(gyroBias[0], gyroBias[1], gyroBias[2]);
    Eigen::Vector3d gyroRawData(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    Eigen::Vector3d gyroCalibratedTriad = gyroCorrection * (gyroRawData - gyroBiasVector);

    msg.angular_velocity.x = gyroCalibratedTriad[0];
    msg.angular_velocity.y = gyroCalibratedTriad[1];
    msg.angular_velocity.z = gyroCalibratedTriad[2];
    // imuPub.publish(msg);
}

void imu_data_calibration::accelCalibration()
{
    Eigen::Vector3d accelBiasVector(accelBias[0], accelBias[1], accelBias[2]);
    Eigen::Vector3d accelRawData(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    Eigen::Vector3d accelCalibratedTriad = accelCorrection * (accelRawData - accelBiasVector);

    msg.linear_acceleration.x = accelCalibratedTriad[0];
    msg.linear_acceleration.y = accelCalibratedTriad[1];
    msg.linear_acceleration.z = accelCalibratedTriad[2];
    // imuPub.publish(msg);
}

/**
 * @brief Madgwick filter designed by Sabastian Madgwick.
 *
 * takes linear accelaration and angular velocity and gives orientation.
 *
 */
void imu_data_calibration::madgwickFilter_()
{
    const geometry_msgs::Vector3 &ang_vel = msg.angular_velocity;
    const geometry_msgs::Vector3 &lin_acc = msg.linear_acceleration;

    ros::Time time = ros::Time::now();

    if (!initialized_ || stateless_)
    {
        geometry_msgs::Quaternion init_q;
        if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc,
                                                      init_q))
        {
            ROS_WARN_THROTTLE(5.0,
                              "The IMU seems to be in free fall, cannot "
                              "determine gravity direction!");
            return;
        }
        filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
    }

    if (!initialized_)
    {
        ROS_INFO("First IMU message received.");
        // initialize time
        last_time_ = time;
        initialized_ = true;
    }

    // determine dt: either constant, or from IMU timestamp
    float dt;
    if (constant_dt_ > 0.0)
        dt = constant_dt_;
    else
    {
        dt = (time - last_time_).toSec();
        if (time.isZero())
            ROS_WARN_STREAM_THROTTLE(
                5.0,
                "The IMU message time stamp is zero, and the parameter "
                "constant_dt is not set!"
                    << " The filter will not update the orientation.");
    }
    last_time_ = time;

    if (!stateless_)
        filter_.madgwickAHRSupdateIMU(ang_vel.x, ang_vel.y, ang_vel.z,
                                      lin_acc.x, lin_acc.y, lin_acc.z, dt);

    publishFilteredMsg();
    if (publish_tf_)
    {
        publishTransform();
    }
}

void imu_data_calibration::publishFilteredMsg()
{
    double q0, q1, q2, q3;
    filter_.getOrientation(q0, q1, q2, q3);

    msg.orientation.w = q0;
    msg.orientation.x = q1;
    msg.orientation.y = q2;
    msg.orientation.z = q3;

    msg.orientation_covariance[0] = orientation_variance_;
    msg.orientation_covariance[1] = 0.0;
    msg.orientation_covariance[2] = 0.0;
    msg.orientation_covariance[3] = 0.0;
    msg.orientation_covariance[4] = orientation_variance_;
    msg.orientation_covariance[5] = 0.0;
    msg.orientation_covariance[6] = 0.0;
    msg.orientation_covariance[7] = 0.0;
    msg.orientation_covariance[8] = orientation_variance_;

    if (remove_gravity_vector_)
    {
        float gx, gy, gz;
        filter_.getGravity(gx, gy, gz);
        msg.linear_acceleration.x -= gx;
        msg.linear_acceleration.y -= gy;
        msg.linear_acceleration.z -= gz;
    }

    // imuPub.publish(msg);
}

void imu_data_calibration::publishTransform()
{
    double q0, q1, q2, q3;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    filter_.getOrientation(q0, q1, q2, q3);
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    if (reverse_tf_)
    {
        transform.header.frame_id = "Fixed_frame";
        transform.child_frame_id = "imu";
        transform.transform.rotation.w = q0;
        transform.transform.rotation.x = -q1;
        transform.transform.rotation.y = -q2;
        transform.transform.rotation.z = -q3;
    }
    else
    {
        transform.header.frame_id = "Fixed_frame";
        transform.child_frame_id = "imu";
        transform.transform.rotation.w = q0;
        transform.transform.rotation.x = q1;
        transform.transform.rotation.y = q2;
        transform.transform.rotation.z = q3;
    }
    tf_broadcaster_.sendTransform(transform);
}