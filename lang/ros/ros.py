
import os

#####################################
def ros_init(pub_topic):
  """check for ROS and start it"""
  try:
    if os.environ["NTA_ROS"]:
      _init(pub_topic)
  except KeyError:
    pass

#####################################
def _init(topic):
  import rospy                        # ros python support
  from std_msgs.msg import String     # import from the ROS standard msg set (*.msg) a msg of type String
  try:
    if os.environ["NTA_ROS_PUB"]:     # Publisher functionality
      pub = rospy.Publisher(topic, String)    # create new publisher which will publish to the topic
      rospy.init_node('talker', anonymous=True)   # init new ROS node called 'talker'
  
  except KeyError:
    pass


#####################################
def publish(str):
  if not rospy.is_shutdown():               # check for ROS node to be killed
    rospy.loginfo(str)                      # print to ROS console
    pub.publish(str)                        # publish the message
