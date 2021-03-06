#!/usr/bin/env python

# NOTE: This code will try to align the IMU's y-axis with NORTH, hence mount the IMU on your robot accordingly.

import rospy
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Twist
from simple_pid import PID
from collections import deque

class MagReader(object):
    def __init__(self):
        self._mag_sub = rospy.Subscriber('/imu/mag', MagneticField, self.mag_cb)  # subscriber to the magnetic field topic from the IMU
        self.mag_data = MagneticField()

    def mag_cb(self, msg):
        self.mag_data = msg.magnetic_field

class NorthOrienter(object):
    def __init__(self):
        self.magreader = MagReader()  # instance of the magnetic field subscriber
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)  # publisher to the wheel motion topic
        self.vel_data = Twist()
        self.max_angular_speed = 0.5  # maximum angular speed at which we want the robot to move
        self.min_angular_speed = 0.1  # minimum angular speed at which we want the robot to move
        # Kp, Kd, Ki - these would need tuning - could be done by adding them to a dynamic reconfigure
        self.kp = 6000
        self.kd = 5000
        self.ki = 5000
        self.tol = 2e-06  # tolerance error for the orientation
        self.target = 5e-05  # target magnetic field value in the y-axis, in Teslas. May also need adjusting depending on the magnetometer used
        self.orientation_estimation_list = deque(maxlen=5)  # a deque allows us to only store 'maxlen' number of parameters in a list at any given time. When we add a new one, the last one is dropped
        self.r = rospy.Rate(10)  # rate of 10Hz - 10 times per second

    def orientate_robot(self):
        # PID controller:
        orientation_pid = PID(self.kp, self.ki, self.kd, self.target)
        orientation_pid.output_limits = (-self.max_angular_speed, self.max_angular_speed)  # setting the limitis for the angular speeds that will be output by the PID control
        if (abs(self.magreader.mag_data.x) > self.tol):  # If the x-reading of the IMU magnetometer is above a certain error, we apply the PID control on the angular_speed
            # Given that IMU data can be a bit noisy, we create a running mean of the last 5 readings to base our PID control on the average of these readings - this smoothens the motion
            self.orientation_estimation_list.append(self.magreader.mag_data.y)  # Negative to ensure that we turn in the correct direction - left if we're pointing East, and right if we are pointing West
            orientation_running_mean = float(sum(self.orientation_estimation_list))/float(len(self.orientation_estimation_list))
            angular_speed = orientation_pid(orientation_running_mean)

            # We will sometimes get calculated angular speeds that are so low that the wheels actually don't turn on the terrain and we never reach the North orientation, we just get close to it. Hence, we set a minimum angular speed - this would also have to be tuned for the robot and the terrain.
            if angular_speed < self.min_angular_speed:
                angular_speed = self.min_angular_speed
            
            # Here we have to make an adjustment, because mag_data.y is minimum at the South and maximum at the North, increasing equally clockwise and anticlockwise, hence angular_speed is always going to be positive. To move anticlockwise when the robot is pointing towards the West (ie when mag_data.x < 0), we have to make the angular_speed negative
            if self.magreader.mag_data.x < 0:  # ie if the front of the robot is angled towards the West
                angular_speed = -angular_speed

            self.vel_data.angular.z = angular_speed
            self._vel_pub.publish(self.vel_data)  # publish the calculated angular_speed to the '/cmd_vel' topic

        elif (abs(self.magreader.mag_data.x) < self.tol) and (self.magreader.mag_data.y > 0):  # If the robot is pointing towards the North then we stop its rotation
            self.vel_data.angular.z = 0
            self._vel_pub.publish(self.vel_data)

        #  We've left a case untouched - if (abs(self.magreader.mag_data.x) < self.tol) and (self.magreader.mag_data.y < 0) - this means that the robot is practically fully aligned with the South direction. If this happens, by skipping it we neither bring the robot to a stop, nor do we change the angular_speed, hence the robot continues moving at the
        #  angular_speed at which it entered this "window". This is a form of hysteresis, added here to prevent rapid switching of the angular_speed when the robot goes through the state of looking South - if not it could start jittering in place.
        #  However, we need to consider the case when the robot is looking South and it isn't moving. This could happen at startup, and if it happens we have to tell the robot to start moving:
        else:
            if self.vel_data.angular.z == 0:
                self.vel_data.angular.z = self.max_angular_speed
                self._vel_pub.publish(self.vel_data)

        print('Robot angular speed: ' + str(self.vel_data.angular.z))  # We print the angular_speed of the robot for debugging purposes
        self.r.sleep()  # wait so that robot motion is smoother

if __name__ == '__main__':
    rospy.init_node('orient_north', anonymous=False)
    northorienter = NorthOrienter()
    rospy.wait_for_message('/imu/mag', MagneticField, timeout=10)  # wait for an initial message of type MagneticField to be published to the /imu/mag topic that we will be listening to. Timeout of 10s
    while not rospy.is_shutdown():
        try:
            northorienter.orientate_robot()
        except rospy.ROSInterruptException:
            pass
