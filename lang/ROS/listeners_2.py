#!/bin/env python2

""" This example demonstrates two publishers join in 1 listener node """

# publisher 2, run this node 2nd
from nupic.encoders.extras.ros import ROSEncoder as ROS
from std_msgs.msg import UInt16
import rospy

def talker():
  publisher2 = ROS(1, "topic2", "Publisher", UInt16)

  i = 0
  while not rospy.is_shutdown(): 
    publisher2.encode(i)
    i += 1
    rospy.sleep(2.0)



if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass

