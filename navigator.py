#!/usr/bin/env python3
"""
Go-to-goal navigation using /odom for robot state and /beacon/pose for target.

Robot state:
  - /odom (nav_msgs/msg/Odometry)  -> x, y, yaw

Target:
  - /beacon/pose (geometry_msgs/msg/Pose) -> x, y
    IMPORTANT: for correct behavior, beacon pose must be expressed in the SAME frame as /odom.

Command:
  - /myrobot/cmd_vel (geometry_msgs/msg/Twist) -> linear.x and angular.z

Control (didactic, robust):
  - Compute bearing to target: theta_T = atan2(ey, ex)
  - Angular error: e_theta = wrap_to_pi(theta_T - theta)
  - If |e_theta| is large -> rotate in place
  - Else -> drive forward proportional to distance (with saturation)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Quaternion (ROS ordering x,y,z,w) -> yaw (rad)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle: float) -> float:
    """Normalize angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# class GoToBeaconOdom(Node):
#     def __init__(self):
#         super().__init__("go_to_beacon_odom")

#         # -------- Parameters --------
#         self.declare_parameter("control_rate_hz", 10.0) # rate of control law

#         self.declare_parameter("k_theta", 1.8)          # Angular speed control gain
#         self.declare_parameter("k_d", 0.8)              # Linear speed control gain

#         self.declare_parameter("v_max", 0.6)            # max linear speed (m/s)
#         self.declare_parameter("omega_max", 1.5)        # max angular speed (rad/s)

#         self.declare_parameter("goal_tolerance", 0.25)  # m
#         self.declare_parameter("theta_align_deg", 20.0) # deg, start driving when within this
#         self.declare_parameter("min_data_age_sec", 5.0) # safety watchdog

#         self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
#         self.k_theta = float(self.get_parameter("k_theta").value)
#         self.k_d = float(self.get_parameter("k_d").value)
#         self.v_max = float(self.get_parameter("v_max").value)
#         self.omega_max = float(self.get_parameter("omega_max").value)
#         self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
#         self.theta_align = math.radians(float(self.get_parameter("theta_align_deg").value))
#         self.min_data_age_sec = float(self.get_parameter("min_data_age_sec").value)

#         # -------- I/O --------
#         self.cmd_pub = self.create_publisher(Twist, "/myrobot/cmd_vel", 10)

#         #only one message: we don't need to react to old data
#         self.create_subscription(Odometry, "/odom", self.on_odom, 10)
#         self.create_subscription(Pose, "/beacon/pose", self.on_beacon_pose, 10)

#         # -------- pos 
#         self.robot_x = 0.0
#         self.robot_y = 0.0
#         self.robot_theta = 0.0
 
#         #we will use the following to check for staleness (old messages - do not react to those)
#         self.beacon_pose = None
#         self.beacon_time = None  
#         self.odom_time   = None
#         # -------- Timer for control law --------
#         self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_step)

#         self.get_logger().info(
#             "GoToBeaconOdom running. Subscribing: /odom, /beacon/pose. Publishing: /myrobot/cmd_vel."
#         )

class GoToBeaconOdom(Node):
    def __init__(self):
        super().__init__("go_to_beacon_odom")

        # -------- Parameters --------
        self.current_distance = 100
        self.distance_threshold = 50
        self.state = "GO_TO_GOAL"

        self.target_x = 2
        self.target_y = 2

        self.declare_parameter("control_rate_hz", 10.0) # rate of control 

        self.declare_parameter("k_theta", 1.8)          # Angular speed control gain
        self.declare_parameter("k_d", 0.8)              # Linear speed control gain

        self.declare_parameter("v_max", 0.6)            # max linear speed (m/s)
        self.declare_parameter("omega_max", 1.5)        # max angular speed (rad/s)

        self.declare_parameter("goal_tolerance", 0.25)  # m
        self.declare_parameter("theta_align_deg", 20.0) # deg, start driving when within this
        self.declare_parameter("min_data_age_sec", 5.0) # safety watchdog

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.k_theta = float(self.get_parameter("k_theta").value)
        self.k_d = float(self.get_parameter("k_d").value)
        self.v_max = float(self.get_parameter("v_max").value)
        self.omega_max = float(self.get_parameter("omega_max").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.theta_align = math.radians(float(self.get_parameter("theta_align_deg").value))
        self.min_data_age_sec = float(self.get_parameter("min_data_age_sec").value)

        # -------- I/O --------
        self.cmd_pub = self.create_publisher(Twist, "/velocity_CRJG", 10)

        #only one message: we don't need to react to old data
        self.create_subscription(Odometry, "/position_CRJG", self.on_position, 10)
        self.create_subscription(Float32, "/distance_CRJG", self.on_distance, 10)
        #fixed X Y location
        # self.create_subscription(Pose, "/beacon/pose", self.on_beacon_pose, 10)

        # -------- pos 
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
 
        #we will use the following to check for staleness (old messages - do not react to those)
        # self.beacon_pose = None
        self.beacon_time = None  
        self.odom_time   = None

        # -------- Timer for control law --------
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_step)

        self.get_logger().info(
            "GoToBeaconOdom running."
        )

    def on_position(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w

        self.robot_theta = yaw_from_quaternion(x, y, z, w)
        self.odom_time = self.get_clock().now()

    def on_distance(self, msg):
        self.get_logger().info(f"New Sonar:{d:.3f} cm) -> stop")
        self.current_distance = msg.data

    # def on_beacon_pose(self, msg: Pose) -> None:
    #     self.beacon_pose = msg
    #     self.beacon_time = self.get_clock().now()

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())
        

    def control_step(self) -> None:
        # Calculate Vector to Goal position
        ex = self.target_x - self.robot_x
        ey = self.target_y - self.robot_y
        dist_to_goal = math.sqrt(ex**2+ey**2)

        if dist_to_goal < self.goal_tolerance:
            self.cmd_pub_publish(twist()) # Stop, Goal Reached
            self.get_logger().info("Goal Reached!")
            return

        if self.current_distance < self.distance_threshold:
            self.state = "AVOID_OBSTACLE"
        elif self.state == "AVOID_OBSTACLE" and self.current_distance > (self.distance_threshold + 0.5):
            self.state = "GO_TO_GOAL"

        if self.state == "AVOID_OBSTACLE":
                v = 0.0 #forward speed
                omega = 0.5 #turning speed
                self.get_logger().info("Obstacle Detected, Turning...")
        else:
            theta_T = math.atan2(ey, ex)
            e_theta = wrap_to_pi(theta_T - self.robot_theta)
            omega = clip(self.k_theta * e_theta, -self.omega_max, self.omega_max)
            # Rotate-in-place until roughly aligned
            if abs(e_theta) > self.theta_align:
                v = 0.0
            else:
                v = clip(self.k_d * d, 0.0, self.v_max)

        # slow down near goal for smooth stop
        slow_radius = 0.4
        if dist_to_goal < slow_radius:
            v *= d / slow_radius

        # Publish Twist
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToBeaconOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
