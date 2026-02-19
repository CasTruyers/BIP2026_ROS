githubs:
ROS2 Navigation: https://github.com/jorgebarreiros-aet/navigation
ROS2 (Janusz): https://github.com/JanuszJakubiak/bip2026_ws

Test Commands:
### Goal coordinate
ros2 topic pub -1 /goal_point geometry_msgs/msg/Point "{x: 2.0, y: 2.0, z: 0.0}"
### Robot position
ros2 topic pub -1 /position_CRJG nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 0.0, y: 0.0}}}}"
### Ultrasound sensor distance
ros2 topic pub /distance_CRJG std_msgs/msg/Float32 "{data: 20.0}"
### Robot position with orientation (angular) at 45 degrees in quaternion format.
ros2 topic pub -1 /position_CRJG nav_msgs/msg/Odometry "{pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.382, w: 0.924}}}}"
