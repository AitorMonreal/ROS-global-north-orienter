#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField

class MagSubscriber(object):
    def __init__(self):
        self._mag_sub = rospy.Subscriber("/imu/mag", MagneticField, self.mag_cb)
        self._mag_data = MagneticField()

    def mag_cb(self, msg):
        self._mag_data = msg.magnetic_field
    
    def get_mag_data(self):
        return self._mag_data


# To debug - If this executable script is ran independently through 'rosrun' we will publish the magnetometer data 10 times per second, all x, y and z data. 
# By aligning the IMU in the North direction, we can see that the x-value on the magnetic field reading approaches 0. Hence this is what we are looking for.
if __name__ == "__main__":
    rospy.init_node("mag_listener", anonymous=False)
    magsubscriber = MagSubscriber()
    while not rospy.is_shutdown():
        try:
            print(magsubscriber.get_mag_data())
            rospy.Rate(10).sleep()
        except rospy.ROSInterruptException:
            pass