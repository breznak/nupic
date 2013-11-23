# ROS (Robot Operating System) 

http://wiki.ros.org/

Support in NuPIC is provided by exposing data flowing between encoders-->SP-->TP-->classifier - this is done 
with help of Passthru encoder (which just moves data around), this encoder is wrapper by ROS logic and provides 
a ROSEncoder. The ROSEncoder can create ROS-Nodes of two types - Publisher and Subscriber. 

Publisher: 
- specify a topic and any data going though this encoder will also be broadcasted on this topic. 
Example publishers.py broadcasts all parts of spatial pooler's chain and allows eg their plotting, etc. 

Raw input --> Publisher("input")--> encoder --> Publisher("encoded") --> SP --> Publisher("SDR")


 
