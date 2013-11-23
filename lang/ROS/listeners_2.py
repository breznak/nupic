#!/bin/env python2

""" This example demonstrates two publishers join in 1 listener node """

# publisher 2, run this node 2nd
from nupic.encoders.extras.ros import ROSEncoder as ROS
from std_msgs.msg import UInt16
import rospy

publisher2 = ROS(1, "topic2", "Publisher")

i = 0
while not rospy.is_shutdown(): 
  publisher2.publish(i)
  i += 1
  rospy.sleep(2.0)


