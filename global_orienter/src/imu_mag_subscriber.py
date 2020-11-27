#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField

class MagSubscriber(object):
    def __init__(self):
        self._mag_sub = rospy.Subscriber("/imu/mag", MagneticField, self.)
        self._mag_data = MagneticField()

    def mag_cb(self, msg):
        self._mag_data = msg.magnetic_field
    
    def get_mag_data(self):
        return self._mag_data
    
if __name__ == "__main__":
    rospy.init_node("mag_listener", anonymous=False)
    magsubscriber = MagSubscriber()
    while not rospy.is_shutdown():
        try:
            magsubscriber.get_mag_data()
        except rospy.ROSInterruptException:
            pass