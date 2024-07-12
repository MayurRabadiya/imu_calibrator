## IMU Calibration Node

### Overview
This package sensor model for Imu calibrated data is implemented in imu_calibration_node. This package subscribes raw data from imu and after filtering publish calibrated data which will use in further process. This package also contains madgwick's filter for oriention. Filter is merged with imu_calibration_node.

### Related papers and existing software

 - The Madgwick's filter algorithm that is used in this project is written by Sebastian Madgwick in
*[" An efficient orientation filter for inertial and inertial/magnatic sensor arrays"](https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf)* paper in 2014 International Symposium.
 - Also, D. Tedaldi, A. Pretto, and E. Menegatti publish *["A Robust and Easy to Implement Method for Imu Calibration Without External Equipments“](https://ieeexplore.ieee.org/document/6907297)* paper in 2014 IEEE
International Conference on Robotics and Automation.

- Source of the Madgwick's filter is given [here](https://github.com/CCNYRoboticsLab/imu_tools).

### How to use package

==> Changes in imu_calibration.launch file:
    
    --> If you want to use Xsens Mti IMU: make "xsens_mti" = "true" and "lsm6dox" = "false".
    --> If you want to use lsm6dox IMU: make "xsens_mti" = "false" and "lsm6dox" = "true".
        NOTE: Don't make both IMU true/false simultaneously in single launch file. 
    
    --> For calibrated data: make "calibration" = "true".
    --> For uncalibrated data: make "calibration" = "false".


Calibration node will subscribe data from Arduino rossserial for lsm6dox and for xsens_mti, lsm_mti drive node has to be run.
Subscriber: GETjag/imu/data   //for xsens_mti
            imu/data/raw  //for lsm6dox from arduino.

Publisher: GETjag/imu/data_new 


Command: roslaunch imu_calibration imu_calibration.launch


--> There is another launch file is inside launch folder to use muliple IMUs with calibration node.
--> Make sure to remap topic name inside imu_calibration_2.launch file according to your use.

--> Inside config folde three .yaml files are there for all three IMUs. Change name of file in launch file as per use.


### Directory Layout
```
imu_calibration
│   README.md
│   package.xml
│   CMakeLists.txt
│
└───config
   │   CalibrationConfiguration_new_lsm.yaml        # calibration parameter for new lsm6dsox imu        
   │   CalibrationConfiguration_old_lsm.yaml        # calibration parameter for old lsm6dsox imu
   │   CalibrationConfiguration_xsens.yaml          # calibration parameter for xsens_MTi imu

│
└───include
│
└───launch
    │   imu_calibration.launch                      
    │   imu_calibration_2.launch                    # To run 2nd imu simultaneously with 1st
|
└───src
```

