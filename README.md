moving_objects is based on http://wiki.ros.org/find_moving_objects for ROS1 and the Github repo is https://github.com/andreasgustavsson/find_moving_objects. ROS2 commands were added and the code was adapted to be used for one lidar sensor with laserScan input. Also, the global position info was added to the message as well as the corresponding functions for that functionality. 

To use the node, pointcloud_to_laserscan ROS2 node is needed for the PointCloud to LaserScan conversion. 
