# IMU Calibrator
Hello all! This project is about simplifying the data collection process of the IMU calibration algorithm for the users and getting more accurate results with the help of GUI. The calibration algorithm to work, there should be an initialization period where IMU stays static. Afterward, there should be several consecutive motion and static intervals to sample data. In the motion intervals, the position of the IMU should be changed, and in the static intervals, the IMU should be held still. It may be tricky to understand when to change position and wait. In order to simplify this process, we introduce a GUI which tells users when to move IMU and when to wait, and it is adjustable to user needs. In the end, it is more convenient to use GUI since it will define motion and static intervals for users precisely.


## imu_calibrater:
This package is about simplifying the data collection process of the IMU calibration algorithm for the users and getting more accurate results with the help of GUI. 
    
## imu_calibration: 
This package sensor model for Imu calibrated data is implemented in imu_calibration_node. This package subscribes raw data from imu and after filtering publish calibrated data which will use in further process. This package also contains madgwick's filter for oriention. Filter is merged with imu_calibration_node.

To get more informatio please see the README file in the perticuler directory.

```
imu_calibration
│   README.md

imu_calibrater
│   README.md
```
