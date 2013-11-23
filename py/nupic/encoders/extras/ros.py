
from nupic.encoders.passthru import PassThruEncoder as I

class ROSEncoder(I):
  """ ROS """

  def __init__(self, n, topic, nodeType, format="String", name="ROS"):
    """params:
         n -- #bits of input (== also #bits of output);
         topic -- a "string" for Publisher, a list of strings for Listener;
	 nodeType -- type of ROS node - values: "Publisher", "Listener"
         format -- (string) type of the ROS msg, see http://wiki.ros.org/std_msgs
    """
    super(ROSEncoder, self).__init__(n, w=None, multiply=1, name=name, forced=True)
    self.type = nodeType
    self.topic = topic
    self.format = format

    if nodeType not in ["Listener", "Publisher"]:
      raise Exception("ROS: nodeType must be one of \"Listener\", \"Publisher\" ")

    try:
      import rospy
      from ros_msgs.msg import *
    except:
      raise Exception("Couldn't import ROS.")
   
  
