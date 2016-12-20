
# Superquadrics based grasping

Fitting superquadrics to objects in clutter:

(For now) Only segmentation part is uploaded due to copyright. The segmentation process 
uses a ROS node for creating supervoxel and then lccp segmentation is implemented 

The node takes supervoxel and lccp parameters, also additional parameters for considering the objects to be lying on the table.
The default openni topic is /camera/depth_registered/point (for the real kinect)

Change this if you have diferent topic for your openni device
