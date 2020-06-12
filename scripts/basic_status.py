#!/usr/bin/env python
from argparse import ArgumentParser
import rospy
import rostopic
from roslib.message import get_message_class
from sensor_msgs.msg import BatteryState

"""
This script creates a ROS node to publish basic RobofleetStatus messages.
Automatically detects:
- sensor_msgs/BatteryState
- jackal_msgs/Status
- f1tenth_course/CarStatusMsg
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

def sub_type(subs, msg_type_str, msg_callback):
    Msg = get_message_class(msg_type_str)
    if Msg is None:
        rospy.loginfo("Could not resolve " + msg_type_str)
        return False
    
    topic = find_topic(Msg)
    if topic is None:
        rospy.loginfo("No publisher for " + msg_type_str)
        return False

    subs.append(rospy.Subscriber(topic, Msg, msg_callback))
    rospy.loginfo("Subscribed to " + msg_type_str)
    return True

def sub_battery_state(subs, data):
    def callback(msg):
        data["battery_level"] = msg.percentage
    return sub_type(subs, "sensor_msgs/BatteryState", callback)

def sub_jackal(subs, data):
    def callback(msg):
        pass
    return sub_type(subs, "jackal_msgs/Status", callback)

def sub_f1tenth(subs, data):
    def callback(msg):
        pass
    return sub_type(subs, "f1tenth_course/CarStatusMsg", callback)

def sub_auto(subs, data):
    if sub_jackal(subs, data):
        return
    if sub_f1tenth(subs, data):
        return
    if sub_battery_state(subs, data):
        return
    rospy.loginfo("Using provided status values.")

def main(ns):
    rospy.init_node("basic_status", anonymous=True)
    subs = []
    data = {
        "battery_level": ns.battery_level,
        "is_ok": ns.is_ok,
        "status": ns.status,
        "location": ns.location
    }

    if ns.auto:
        sub_auto(subs, data)
    
    rate = rospy.Rate(5)
    status_pub = rospy.Publisher("status", RobofleetStatus, queue_size=1)
    while not rospy.is_shutdown():
        status = RobofleetStatus()
        status.battery_level = data["battery_level"]
        status.is_ok = data["is_ok"]
        status.status = data["status"]
        status.location = data["location"]
        status_pub.publish(status)
        rospy.loginfo("Published")
        rate.sleep()

if __name__ == "__main__":
    ap = ArgumentParser("basic_status.py")
    ap.add_argument("--auto", "-a", action="store_true",
        help="Automatically detect status")
    ap.add_argument("--battery-level", nargs="?", type=float, default=0, 
        help="Default battery level")
    ap.add_argument("--not-ok", dest="is_ok", action="store_false", 
        help="Set is_ok to false")
    ap.add_argument("--status", nargs="?", default="n/a", 
        help="Default status message")
    ap.add_argument("--location", nargs="?", default="n/a",
        help="Default location message")
    ns = ap.parse_args()
    main(ns)
