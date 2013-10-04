
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
  from std_msgs.msg import String     # import from the ROS standard msg set (*.msg) a msg of type String #TODO: use numpy array
  try:
    if os.environ["NTA_ROS_PUB"]:     # Publisher functionality
      pub = rospy.Publisher(topic, String)    # create new publisher which will publish to the topic
      rospy.init_node('talker', anonymous=True)   # init new ROS node called 'talker'
      publish = _publish  # hack: if NTA_ROS_PUB is set, assign a _publish fn that actually does something, otherwise just dummy skip
  except KeyError:
    pass


#####################################
def _publish(str):
  """the publisher functionality that publishes string str on topic (given in ros_init)"""
  if not rospy.is_shutdown():               # check for ROS node to be killed
    rospy.loginfo(str)                      # print to ROS console
    pub.publish(str)                        # publish the message


#####################################
def publish(str):
  """a dummy fn that just skips"""
  pass


#####################################
#def callback(data):                         # callback funciton - workload for listener()
#  rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data) # write to ROS console
    

#####################################
def listener(topic,callbackFn):
  """a counter-part to publisher, runs in loop!, you need to define callbackFn"""
  # in ROS, nodes are unique named. If two nodes with the same
  # node are launched, the previous one is kicked off. The 
  # anonymous=True flag means that rospy will choose a unique
  # name for our 'talker' node so that multiple talkers can
  # run simultaenously.
  rospy.init_node('listener', anonymous=True)     # create new ROS node
  # create subscriber for messages of type String on the topic & assign callback fcn
  rospy.Subscriber(topic, String, callbackFn)   
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()


