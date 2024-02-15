*GTest Framework for ROS2*

Created a package named "tortoisebot_waypoints" then inside this package created a C++ program named:
tortoisebot_action_server.cpp that allows the robot to follow provided waypoints. 

After that created ROS Node Level tests using GTest:
  
  1: Check that the end position [X,Y] of the robot is the expected one bases on the goal sent. By default the goal is
     a Point msg with value -> x = 0.1, y = 0.1. 

  2: Check that the end rotation [Yaw] of the robot is the expected one based on the goal sent. Same as before for both tests.

![imagen](https://github.com/Romu10/GTest-Framework-for-ROS2/assets/132951300/94de91f3-8db6-4c64-b9b2-83d688e7ab3b)

![imagen](https://github.com/Romu10/GTest-Framework-for-ROS2/assets/132951300/93c48567-a4c9-4ef2-bde9-2cdf6d070446)
