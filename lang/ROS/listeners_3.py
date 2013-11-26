#!/bin/env python2

""" This example demonstrates two publishers join in 1 listener node """

# listener node, run this node last
from nupic.encoders.extras.ros import ROSSubscriber as ROS
from nupic.encoders.extras.ros import ROSPublisher as PUB
from std_msgs.msg import UInt16, String
import rospy


def listen():
# use one listener subscribed to multiple topics - TimeSync
  listener = ROS(2, ["topic1", "topic2"], [UInt16,UInt16], callback, postListenTopic=None, postListenFormat=String)

########################################################################

def callback(data1, data2):
  str= "recieved: ", data1, " + ", data2
  print str
  return str # for postListener Publisher's data

if __name__ == '__main__':
     try:
         listen()
     except rospy.ROSInterruptException:
         pass

