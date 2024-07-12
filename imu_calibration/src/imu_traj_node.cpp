#include <imu_calibration/imu_traj.h>

int main (int argc, char *argv[])
{
    ros::init (argc, argv, "imu_traj");
    ROS_INFO("Imu Trajectory node running");
    Imu_traj imu_traj;
    // ros::Rate r (101);
    ros::spin();
    return 0;
}
