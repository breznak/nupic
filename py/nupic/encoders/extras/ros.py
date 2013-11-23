
from nupic.encoders.passthru import PassThruEncoder as I
try:
  import rospy
  from std_msgs.msg import *
except:
  print "Couldn't import ROS."
  raise


class ROSPublisher(I):
  """ ROS Publisher node wraped around PassThru encoder for CLA"""

  def __init__(self, n, topic, msgFormat, name="ROS"):
    """params:
         n -- #bits of input (== also #bits of output);
         topic -- a "string" for Publisher;
         msgFormat -- (obj) type of the ROS msg, see http://wiki.ros.org/std_msgs
    """
    super(ROSPublisher, self).__init__(n, w=None, multiply=1, name=name, forced=True)
    self.topic = topic
    self.format = msgFormat

    self.pub = rospy.Publisher(self.topic, self.format)    # create new publisher which will publish to the topic
    rospy.init_node("Publisher_"+self.topic, anonymous=True)   # init new ROS node called 'talker'

  # override parent
  def encode(self, input):
    if rospy.is_shutdown():               # check for ROS node to be killed
      raise Exception("ROS is down")
    rospy.loginfo(input)                      # print to ROS console
    self.pub.publish(self.format(input))                        # publish the message
    # actually only pass the data further:
    return input



class ROSSubscriber(I):
  """ROS Subscriber node wrapped around PassThru encoder for CLA"""

  def __init__(self, n, topic, msgFormat, listenerCallbackFn, postListenTopic=None, name="ROS"):
    """params:
         n -- #bits of input (== also #bits of output);
         topic -- a "string" for Publisher, a list of strings for Listener;
         msgFormat -- (obj) type of the ROS msg, see http://wiki.ros.org/std_msgs
         listenerCallbackFn -- a function, that is executed each time listener recieves an input;
         postListenTopic  -- (optional) (string) topic to which data are reposted after recieving by the listener. 

       Warning: Listener node is passive - it sits and waits for data to come. After calling the constructor(), Listener will 
                start listening in an infinite loop - no code after it will be executed.
    """
    super(ROSListener, self).__init__(n, w=None, multiply=1, name=name, forced=True)
    self.topic = topic
    self._postTopic = postListenTopic
    self.format = msgFormat
    self._callback = listenerCallbackFn
    self.pub = None

    if self._postTopic is not None:
      self.pub = rospy.Publisher(self._postTopic, self.format) # to this channel we'll send the data after listener's callback is finished
    rospy.init_node("Listener_"+topic, anonymous=True)     # create new ROS node
    # create subscriber for messages of type String on the topic & assign callback fcn
    rospy.Subscriber(self.topic, self.format, self.callback)
    # spin() simply keeps python from exiting until this node is stopped - loops forever! 
    #rospy.spin() 
    # call manually from the application, eg use TimeSync etc.. 

  def loop(self):
    rospy.spin()


  def callback(self,data):                         
    """callback funciton - workload for listener()"""
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data) # write to ROS console
    self._callback(data.data) # call the user-defined function
    if self._postTopic is not None:
      self.pub.publish(self.format(data.data))  # publish further

