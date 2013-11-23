# ROS (Robot Operating System) 

http://wiki.ros.org/

Support in NuPIC is provided by exposing data flowing between encoders-->SP-->TP-->classifier - this is done 
with help of Passthru encoder (which just moves data around), this encoder is wrapper by ROS logic and provides 
a ROSEncoder. The ROSEncoder can create ROS-Nodes of two types - Publisher and Subscriber. 

Publisher: 
- specify a topic and any data going though this encoder will also be broadcasted on this topic. 
Example publishers.py broadcasts all parts of spatial pooler's chain and allows eg their plotting, etc. 

Raw input --> Publisher("input")--> encoder --> Publisher("encoded") --> SP --> Publisher("SDR")


Listener/Subscriber 
- is little more complicated; subscribes to a topic and listens to all msgs recieved on it. 
This is actually a passive element, it's waiting in a loop and when a new message arrives, it'll call the callback function
which the user provided and which should do appropriate logic with the data in msg. 

For convenience, an optional Publisher was added, which is called at the very end of callback function and publishes to 
`postListenTopic`. This way, it's possible to recieve data, process it and pass further. 

Example in listeners_{1,2,3}.py: 

publisher1 ---\
               \____ listener --> CLA processing --> publisher "news"
               /
publisher2 ---/


