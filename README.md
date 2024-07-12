# Project Group "Hardware - Mobile Mapping Unit" (Summer Semester 2022)

## Group Members

* [Bhagyashree Mane](bhagmane@campus.uni-paderborn.de)
* [Mayur Rabadiya](rmayur@campus.uni-paderborn.de)
* [Ronak Gandhi](rgandhi@campus.uni-paderborn.de)



# imu_calibrater:
    This package is about simplifying the data collection process of the IMU calibration algorithm for the users and getting more accurate results with the help of GUI. 
    
# imu_calibration: 
    This package sensor model for Imu calibrated data is implemented in imu_calibration_node. This package subscribes raw data from imu and after filtering publish calibrated data which will use in further process. This package also contains madgwick's filter for oriention. Filter is merged with imu_calibration_node.

To get more informatio please see the README file in the perticuler directory.

imu_calibration
│   README.md

imu_calibrater
│   README.md
