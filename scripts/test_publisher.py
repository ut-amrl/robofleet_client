#!/usr/bin/env python
import math
import random
import rospy
from roslib.message import get_message_class

seed = random.random() * 100000

def talker():
    # avoid that whole "make a ROS package" issue
    # make sure ROS_PACKAGE_PATH contains amrl_msgs
    RobofleetStatus = get_message_class("amrl_msgs/RobofleetStatus")
    if RobofleetStatus is None:
        raise Exception("Ensure that amrl_msgs is on ROS_PACKAGE_PATH")
    Localization2DMsg = get_message_class("amrl_msgs/Localization2DMsg")
    Odometry = get_message_class("nav_msgs/Odometry")

    status_pub = rospy.Publisher("status", RobofleetStatus, queue_size=1)
    odom_pub = rospy.Publisher("odometry/raw", Odometry, queue_size=1)
    loc_pub = rospy.Publisher("localization", Localization2DMsg, queue_size=1)

    rospy.init_node("test_publisher", anonymous=True)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        t = rospy.get_time() + seed

        rf_status = RobofleetStatus()
        rf_status.battery_level = math.sin(t / 5) * 0.5 + 0.5
        rf_status.is_ok = True
        rf_status.location = "cyberspace"
        rf_status.status = "Testing"

        odom = Odometry()
        odom.pose.pose.position.x = math.sin(t / 10) * 5 + 5
        odom.pose.pose.position.y = math.sin(t / 10 + 0.5) * 5 + 5
        odom.pose.pose.position.z = math.sin(t / 10 + 1) * 5 + 5
        odom.twist.twist.linear.x = 1
        odom.twist.twist.linear.y = 2
        odom.twist.twist.linear.z = 3

        loc = Localization2DMsg()
        loc.map = "UT_Campus"
        loc.pose.x = math.sin(t / 50) * 100 + 100
        loc.pose.y = math.cos(t / 50) * -100 - 100
        loc.pose.theta = t / 10

        rospy.loginfo("publishing")
        status_pub.publish(rf_status)
        odom_pub.publish(odom)
        loc_pub.publish(loc)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
