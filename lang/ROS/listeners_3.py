#!/bin/env python2

""" This example demonstrates two publishers join in 1 listener node """

# listener node, run this node last
from nupic.encoders.extras.ros import ROSEncoder as ROS
from std_msgs.msg import UInt16
import rospy

listener1 = ROS(1, "topic1", "Listener", listenerCallbackFn=callback1, postListenTopic="news")
listener2 = ROS(1, "topic2", "Listener", listenerCallbackFn=callback2, postListenTopic="news")


_msg1 = None # msg recieved from Listener1
_msg2 = None # dtto 2

def callback1(data):
  _msg1 = data.data
  pprint()

def callback2(data):
  _msg2 = data.data
  pprint()

def pprint():
  if(_msg1 is not None and _msg2 is not None):
    print "Listener: ", _msg1, " = ", _msg2
    _msg1 = None
    _msg2 = None

  
