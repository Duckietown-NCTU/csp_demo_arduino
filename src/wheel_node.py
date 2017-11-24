#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import BoolStamped, Twist2DStamped

class arduinoROS(object):
    def __init__(self):
        self.dis = 0.0 # distance from ultrasound

        # =========== publisher ===========
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # =========== subscriber ===========
        self.sub_tags = rospy.Subscriber("~result", BoolStamped, self.cbresult)

   # =========== subscribe distance from arduino ===========
    def cbresult(self, msg):
        cmd = Twist2DStamped()
        if ???:
            print "go forward"

        else:
            print "go backward"

if __name__ == "__main__":
    rospy.init_node("arduino_ros", anonymous = False)
    arduino_node = arduinoROS()
    rospy.spin()