
from nupic.encoders.passthru import PassThruEncoder as I

class ROSEncoder(I):
  """ ROS """

  def __init__(self, n, topic, nodeType, msgFormat, name="ROS"):
    """params:
         n -- #bits of input (== also #bits of output);
         topic -- a "string" for Publisher, a list of strings for Listener;
	 nodeType -- type of ROS node - values: "Publisher", "Listener"
         msgFormat -- (obj) type of the ROS msg, see http://wiki.ros.org/std_msgs
    """
    super(ROSEncoder, self).__init__(n, w=None, multiply=1, name=name, forced=True)
    self.type = nodeType
    self.topic = topic
    self.format = msgFormat

    if nodeType not in ["Listener", "Publisher"]:
      raise Exception("ROS: nodeType must be one of \"Listener\", \"Publisher\" ")

    try:
      import rospy
      from ros_msgs.msg import *
    except:
      raise Exception("Couldn't import ROS.")
   
    if self.type == "Publisher":
      self.pub = rospy.Publisher(self.topic, self.format)    # create new publisher which will publish to the topic
      rospy.init_node('talker', anonymous=True)   # init new ROS node called 'talker'
    elif self.type == "Listener":
      pass


  # override parent
  def encode(self, input):
    if rospy.is_shutdown():               # check for ROS node to be killed
      raise Exception("ROS is down")
    rospy.loginfo(input)                      # print to ROS console
    self.pub.publish(input)                        # publish the message
    # actually only pass the data further:
    return input

