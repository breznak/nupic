#!/bin/env python2

""" This example demonstrates two publishers join in 1 listener node """

# publisher 1, run this node 1st
from nupic.encoders.extras.ros import ROSEncoder as ROS
from std_msgs.msg import UInt16
import rospy

publisher1 = ROS(1, "topic1", "Publisher")

i = 0
while not rospy.is_shutdown(): 
  publisher1.publish(i)
  i += 1
  rospy.sleep(2.0)


