#!/bin/env python2

""" This example demonstrates two publishers join in 1 listener node """

# publisher 1, run this node 1st
from nupic.encoders.extras.ros import ROSPublisher as ROS
from std_msgs.msg import UInt16
import rospy

def talker():
  publisher1 = ROS(1, "topic1", UInt16)

  i = 0
  while not rospy.is_shutdown(): 
    publisher1.encode(i)
    i += 1
    rospy.sleep(2.0)



if __name__ == '__main__':     
     try:
         talker()
     except rospy.ROSInterruptException:
         pass
