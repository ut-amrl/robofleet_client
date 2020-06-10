#!/usr/bin/env python
import math
import rospy
from roslib.message import get_message_class

def talker():
    # avoid that whole "make a ROS package" issue
    # make sure ROS_PACKAGE_PATH contains amrl_msgs
    RobofleetStatus = get_message_class("amrl_msgs/RobofleetStatus")

    pub = rospy.Publisher("/test/status", RobofleetStatus, queue_size=1)
    rospy.init_node("status_tester", anonymous=True)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rf_status = RobofleetStatus()
        rf_status.battery_level = math.sin(rospy.get_time() / 5) * 0.5 + 0.5
        rf_status.is_ok = True
        rf_status.location = "cyberspace"
        rf_status.status = "Testing"
        rospy.loginfo(rf_status)
        pub.publish(rf_status)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
