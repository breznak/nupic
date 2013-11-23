
from nupic.encoders.passthru import PassThruEncoder as I
try:
  import rospy
  from std_msgs.msg import *
except:
  print "Couldn't import ROS."
  raise


class ROSEncoder(I):
  """ ROS """

  def __init__(self, n, topic, nodeType, msgFormat, listenerCallbackFn=None, postListenTopic=None, name="ROS"):
    """params:
         n -- #bits of input (== also #bits of output);
         topic -- a "string" for Publisher, a list of strings for Listener;
	 nodeType -- type of ROS node - values: "Publisher", "Listener"
         msgFormat -- (obj) type of the ROS msg, see http://wiki.ros.org/std_msgs
       extra params required for Listener type:
         listenerCallbackFn -- a function, that is executed each time listener recieves an input, takes 1 argument - data;
         postListenTopic  -- (optional) (string) topic to which data are reposted after recieving by the listener. 

       Warning: Listener node is passive - it sits and waits for data to come. After calling the constructor(), Listener will 
                start listening in an infinite loop - no code after it will be executed.
    """
    super(ROSEncoder, self).__init__(n, w=None, multiply=1, name=name, forced=True)
    self.type = nodeType
    self.topic = topic
    self._postTopic = postListenTopic
    self.format = msgFormat
    self._callback = listenerCallbackFn

    if nodeType not in ["Listener", "Publisher"]:
      raise Exception("ROS: nodeType must be one of \"Listener\", \"Publisher\" ")

    if self.type == "Publisher":
      self.pub = rospy.Publisher(self.topic, self.format)    # create new publisher which will publish to the topic
      rospy.init_node('talker', anonymous=True)   # init new ROS node called 'talker'
    elif self.type == "Listener":
      if self._postTopic is not None:
        self.pub = rospy.Publisher(self._postTopic, self.format) # to this channel we'll send the data after listener's callback is finished
      rospy.init_node('listener', anonymous=True)     # create new ROS node
      # create subscriber for messages of type String on the topic & assign callback fcn
      rospy.Subscriber(self.topic, self.format, self.callback)   
      # spin() simply keeps python from exiting until this node is stopped - loops forever! 
      rospy.spin()


  # override parent
  def encode(self, input):
    if rospy.is_shutdown():               # check for ROS node to be killed
      raise Exception("ROS is down")
    rospy.loginfo(input)                      # print to ROS console
    self.pub.publish(self.format(input))                        # publish the message
    # actually only pass the data further:
    return input


  def callback(data):                         
    """callback funciton - workload for listener()"""
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data) # write to ROS console
    self._callback(data.data) # call the user-defined function
    if self._postTopic is not None:
      self.decode(data.data)  # publish further

