
## IMU Calibration GUI

### Overview
Hello all! This project is about simplifying the data collection process of the IMU calibration algorithm for the users and getting more accurate results with the help of GUI. The calibration algorithm to work, there should be an initialization period where IMU stays static. Afterward, there should be several consecutive motion and static intervals to sample data. In the motion intervals, the position of the IMU should be changed, and in the static intervals, the IMU should be held still. It may be tricky to understand when to change position and wait. In order to simplify this process, we introduce a GUI which tells users when to move IMU and when to wait, and it is adjustable to user needs. In the end, it is more convenient to use GUI since it will define motion and static intervals for users precisely.

### Related papers and existing software

 - The calibration algorithm that is used in this project is written by A. Pretto and G. Grisetti in
*["Calibration and Performance Evaluation of Low-Cost Imus"](https://www.imeko.org/publications/tc4-2014/IMEKO-TC4-2014-429.pdf)* paper in 2010.

- Source of the IMU calibration is given [here](https://bitbucket.org/alberto_pretto/imu_tk/src/master/).

### How to use GUI & What you should expect

The GUI should have two main functionalities, one is recording data, and the other is calibrating the IMU. The static initialization duration, number of poses, duration of static and motion intervals, and the file where the recorded data is going to be saved should be given to record data. It should tell the user when to change the position of the IMU and when to wait. The progress can be seen from the progress bar. To calibrate the data, only the file path to be calibrated should be given. In short, users should be able to record and calibrate IMU with two buttons only.


### Directory Layout
```
imu_calibration
│   README.md
│   package.xml
│   CMakeLists.txt
│
└───deps
│   │   imu_tk                           # the calibration algorithm
│
└───gui
    │   CalibrationWidget.cpp            # main source file
    │   CalibrationWidget.h              # header file
    │   CalibrationWidget.ui             # qt design of the GUI
 │
 └───include
 │
 └───launch
    │   gui_test.launch
 │
 └───src
    │   imu_read_data.cpp                 # source file to record data
    │   test_imu_calib.cpp                # source file to calibrate IMU
```
