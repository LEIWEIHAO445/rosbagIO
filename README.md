#############################################
# rosbag Project
# function: read and write data from bag file
#############################################

### history
    2022.12.12     Leiwh     write IMU observation to bag, extract IMU observation and image data from bag file


### 1. Prerequisites

## ROS

## OpenCV


### 2. Building Project

  Enter the *rosbag* folder, open a terminal and input *catkin_make* to build the project

### 2. Running IPSProject

  Execute *rosbag* in *./devel/lib/rosbagIO/* with different parameters to run the project, parameters are following:
  
  |                                                    parameters                                                    |               function               |
  -----------------------------------------------------------------------------------------------------------------------------------------------------------
  |        para 1        |        para 2        |        para 3        |        para 4        |        para 5        |                                      |
        bag filepath         output file path          ros topic                 mode                data type               extract data from bag file
        data filepath        output file path          ros topic                 mode                data type               write data to bag file

  note: mode 0 for extract and 1 for write, data type 0 for IMU data and 1 for image data
  
  Example: *./rosbagIO yourfolder/bagfile your_output_filepath /imu/data 0 0* to extract IMU data from bag file
