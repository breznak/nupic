#!/bin/env python2

"""Example of ROS's publisher nodes. Displays data flow in a SP"""

from nupic.encoders.scalar import ScalarEncoder as Enc
from nupic.encoders.extras.ros import ROSPublisher as ROS
from nupic.research.spatial_pooler import SpatialPooler as SP
from std_msgs.msg import Float32, Int8MultiArray # the ROS type of msgs we'll be sending - float and int8[] 
import rospy
import numpy

def work():
  # initialize:
  data=[1,2,3,4,5]
  layer0 = ROS(1, "input", Float32) # pass thru encoder for raw input
  enc = Enc(3, 1, 5, n=15) # encoder for raw input to bit-array
  layer1 = ROS(enc.getWidth(), "encoded", Int8MultiArray) # publish the encoded bit array
  sp = SP( inputDimensions=enc.getWidth(),
               columnDimensions=10,
               potentialRadius=3,
               potentialPct=0.5,
               globalInhibition=True,
               localAreaDensity=-1.0,
               numActiveColumnsPerInhArea=3.0,
               stimulusThreshold=0,
               synPermInactiveDec=0.05,
               synPermActiveInc=0.1,
               synPermConnected=0.2,
               minPctOverlapDutyCycle=0.001,
               minPctActiveDutyCycle=0.001,
               dutyCyclePeriod=1000,
               maxBoost=10.0,
               seed=-1,
               spVerbosity=0,
          )
  layer2 = ROS(sp.getNumColumns(), "SP", Int8MultiArray) # SDR from the SP

  # run all the layers:
  for d in data:
    i = d
    print "at layer 0, type:",type(i)," msg:",i 
    i = layer0.encode(i)
    i = enc.encode(i)
    i = i.tolist()
    print "at layer 1, type:",type(i)," msg:",i
    i = layer1.encode(i)
    retVal = numpy.array([0]*len(i))
    sp.compute(numpy.array([i]), True, retVal)
    i = retVal.tolist()
    print "at layer 2, type:",type(i)," msg:",i
    i = layer2.encode(i)
    rospy.sleep(2.0)

  # that's it :)  


if __name__ == '__main__':
     try:
         work()
     except rospy.ROSInterruptException:
         pass
