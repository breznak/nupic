
import os

def ros_init():
  """check for ROS"""
  try:
    if os.environ["NTA_ROS"]:
      _init()
  except KeyError:
    pass

def _init():
  pass
