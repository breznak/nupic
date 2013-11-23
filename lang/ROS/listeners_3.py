#!/bin/env python2

""" This example demonstrates two publishers join in 1 listener node """

# listener node, run this node last
from nupic.encoders.extras.ros import ROSEncoder as ROS
from std_msgs.msg import UInt16
import rospy


def listen():
#TODO: use one listener subscribed to multiple topics
  listener1 = ROS(1, "topic1", "Listener", UInt16, listenerCallbackFn=callback1, postListenTopic="news")
  listener2 = ROS(1, "topic2", "Listener", UInt16, listenerCallbackFn=callback2, postListenTopic="news")

########################################################################
# global functions, variables
_msg1 = None # msg recieved from Listener1
_msg2 = None # dtto 2

def callback1(data):
    global _msg1
    _msg1 = data
    print "call1", data
    pprint()

def callback2(data):
    global _msg2
    _msg2 = data
    print "call2", _msg2
    pprint()

def pprint():
    global _msg1
    global _msg2
    if(_msg1 is not None and _msg2 is not None):
      print "Listener: ", _msg1, " = ", _msg2
      _msg1 = None
      _msg2 = None



if __name__ == '__main__':
     try:
         listen()
     except rospy.ROSInterruptException:
         pass

