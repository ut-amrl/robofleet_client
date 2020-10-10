#!/usr/bin/env python
from argparse import ArgumentParser
import rospy
import rostopic
from roslib.message import get_message_class

"""
This script creates a ROS node to publish basic RobofleetStatus messages.
"""

# avoid that whole "make a ROS package" issue
# make sure ROS_PACKAGE_PATH contains amrl_msgs
RobofleetStatus = get_message_class("amrl_msgs/RobofleetStatus")
if RobofleetStatus is None:
    raise Exception("Ensure that amrl_msgs is on ROS_PACKAGE_PATH")

def find_topic(msg_type):
    pubs, _ = rostopic.get_topic_list()
    for topic, topic_type, _ in pubs:
        if topic_type == msg_type:
            return topic
    return None

def main(ns):
    rospy.init_node("basic_status", anonymous=True)

    rate = rospy.Rate(5)
    status_pub = rospy.Publisher("status", RobofleetStatus, queue_size=1)
    while not rospy.is_shutdown():
        status = RobofleetStatus()
        status.battery_level = ns.battery_level
        status.is_ok = ns.is_ok
        status.status = ns.status
        status.location = ns.location
        status_pub.publish(status)
        rospy.loginfo("Published")
        rate.sleep()

if __name__ == "__main__":
    ap = ArgumentParser("basic_status.py")
    ap.add_argument("--battery-level", "-b", nargs="?", type=float, default=0, 
        help="Default battery level")
    ap.add_argument("--not-ok", "-n", dest="is_ok", action="store_false", 
        help="Set is_ok to false")
    ap.add_argument("--status", "-s", nargs="?", default="n/a", 
        help="Default status message")
    ap.add_argument("--location", "-l", nargs="?", default="n/a",
        help="Default location message")
    ns = ap.parse_args()
    main(ns)
