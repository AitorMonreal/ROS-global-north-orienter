# Global North Orienter

This ROS program adjusts the angular speed of a robot based on the magnetometer readings from an IMU mounted on it in order to align the robot with the North from anywhere on the globe (both on the Northern and Southern hemispheres). Check video_showcase.mp4 for a detailed showcase explanation.

## Requirements

For the code to work, the IMU has to be mounted with its y-axis pointing towards the front of the robot, and its x-axis pointing towards the left of the robot.
This project uses a Phidgets IMU. To be able to take its readings into ROS you will have to install the following two packages:

- phidgets_drivers -> contains the drivers that enable the ROS linkage. Install from https://github.com/ros-drivers/phidgets_drivers
- imu_tools -> contains the filters that enable reading magnetometer values directly. Install from https://github.com/ccny-ros-pkg/imu_tools

## Interpreting readings from the magnetometer: 

These are published to the /imu/mag topic using a MagneticField message type. The magnetic_field section of this data is split into x, y, and z. Each of them store the intensity of the magnetic field in their direction in Teslas. When the IMU's y-axis is pointing towards the North it takes its maximum reading, which is approximately 5e-05T. When the y-axis points towards the South it takes its minimum reading: -5e-05T. When the y-axis is pointing towards the West or East, it reads 0. Hence, writing the readings as (x, y) (ignoring z):

- North - (0, 5e-05)
- East  - (5e-05, 0)
- South - (0, -5e-05)
- West  - (-5e-05, 0)

## Steps to run the code:

1. First clone the repo, install the aforementioned 2 IMU packages into your workspace, and launch the phidgets_imu.launch
2. You can now rosrun the imu_mag_print.py to get a better idea of the output of the magnetometer readings. NOTE: the magnetometer will be affected by other magnetic fields and large ferrous structures, so take this into account when using this code.
3. launch the north_orienter.launch file. This outputs the calculated angular_speed of the robot to the /cmd_vel topic using a PID controller. Play around with the controller gains and other code parameters until you are happy with the results.
