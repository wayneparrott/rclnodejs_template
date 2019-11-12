# rplidar_ros_rclnodejs
A ROS2 package that publishes 2D LIDAR scan data emitted by RPLidar A1 - A3 devices. Implemented as a Node.js program in TypeScript on top of the [rclnodejs]() JavaScript ROS2 client interface and rplidar-driver.



As a ROS2 package this project needs to be part of a ROS2 workspace. 

todo:
* defaults, assumptions and limitations
* describe how to extract into a ros2 workspace
* run compiler
* run npm i - this will build js messages
* how to run
* using ros2 cli to send msgs: start_motor, stop_motor, subscribe and echo scan data


https://github.com/robopeak/rplidar_ros/blob/master/src/node.cpp

// influence https://github.com/robopeak/rplidar_ros/blob/master/src/node.cpp
//

// > ros2 service call /start_motor rplidar_ros_rclnodejs/srv/Control
// > ros2 service call /stop_motor rplidar_ros_rclnodejs/srv/Control
// > ros2 service call /reset rplidar_ros_rclnodejs/srv/Control
// > ros2 topic echo /rplidar_scan_data sensor_msgs/msg/LaserScan


ros2 pkg executables rplidar_ros_rclnodejs --full-path
ros2 run rplidar_ros_rclnodejs rplidar_node
ros2 launch rplidar_ros_rclnodejs rplidar_node.launch.py


Things to know before using this ROS2 package:
* uses the rplidar-driver for interacting with rplidar devices. This driver is early in development and only supports the legacy scan mode, i.e., slowest yet very useful scan mode. 
* rplidar devices return a 360 degree scan that will include some low quality or non-useful samples. These samples are removed from the scan data resulting in non-uniform distribution of samples across the 360 degree scan range. The sensor_msgs/LaserScan assumes it's scan samples are uniformly distributed, which they are not. As a result when rendered in RVIZ will redistribute the filtered sample positions over the 360 degrees resulting in a more coarse, lower resolution display. The visual effect is of points bouncing around on the virtual outline of objects in successive scans. This is because number of filtered points from each 360 scan sample frequently varies by a small number. The variation results in the same point's computed position to vary slight across each render scan by RVIZ.

Example, consider this subset of a scan where the device returns 3 samples each 1 ms apart. 
```
        pt0   {angle: 1,                {angle: 1,
	         distance: 1000,           distance: 1000,
	         quality: 15}              quality: 15}
	pt1   {angle: 2,
	         distance: 0,             ***filtered***
	         quality: 0}
	pt2   {angle: 3,                {angle: 3,
	         distance: 1000,           distance: 1200,
	         quality: 15}              quality: 15}
```
After filtering the low quality pt1 sample, RVIZ will render the 2 remaining points evenly distributed over 360 degrees. In the next scan let's assume the pt1 sample is of high quality and is not filtered. This results in 3 points being rendered. Now pt0 and pt2 are shown offset from their previous position. This creates a visual blinking and drifting effect of some points. 

The plan is to use a high sample rate with rplidar-driver as soon as it comes available. But this will only solve 1/2 the problem. The other 1/2 of the problem is that sample filtering will create non-uniform data distribution. To compensate, rather than filtered out low quality samples, they are instead linearly interpolated. This results in higher a resolution map with much less point drift.


