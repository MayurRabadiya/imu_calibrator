#include <imu_calibration/imu_calibration.h>

int main (int argc, char *argv[])
{
    ros::init (argc, argv, "imu_data_calibration");
    ros::NodeHandle nh;
	ros::NodeHandle nhP("~");
    imu_data_calibration ImuCalibration(nh, nhP);
    ros::spin();
    return 0;
}