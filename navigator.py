#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Twist, Point # Added Point for manual goal input

def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def clip(value, lo, hi):
    return max(lo, min(hi, value))

class GoToBeaconOdom(Node):
    def __init__(self):
        super().__init__("go_to_beacon_odom")

        # -------- Navigation State --------
        self.current_distance = 100.0
        self.distance_threshold = 50.0 # Caution: Ensure units match (cm vs m)
        self.state = "GO_TO_GOAL"

        # These will be updated by the /goal_point subscriber
        self.target_x = 0.0 
        self.target_y = 0.0
        self.goal_received = False

        # -------- Parameters --------
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("k_theta", 1.8)
        self.declare_parameter("k_d", 0.8)
        self.declare_parameter("v_max", 0.6)
        self.declare_parameter("omega_max", 1.5)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("theta_align_deg", 20.0)

        self.control_rate_hz = self.get_parameter("control_rate_hz").value
        self.k_theta = self.get_parameter("k_theta").value
        self.k_d = self.get_parameter("k_d").value
        self.v_max = self.get_parameter("v_max").value
        self.omega_max = self.get_parameter("omega_max").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.theta_align = math.radians(self.get_parameter("theta_align_deg").value)

        # -------- I/O --------
        self.cmd_pub = self.create_publisher(Twist, "/velocity_CRJG", 10)
        
        self.create_subscription(Odometry, "/position_CRJG", self.on_position, 10)
        self.create_subscription(Float32, "/distance_CRJG", self.on_distance, 10)
        # New: Subscriber for manual goal input from terminal
        self.create_subscription(Point, "/goal_point", self.on_goal, 10)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.odom_received = False

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_step)
        self.get_logger().info("Navigation Node Initialized. Waiting for Goal on /goal_point...")

    def on_position(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        self.get_logger().info(f"Odom: x={self.robot_x:.2f}, y={self.robot_y:.2f}")

        q = msg.pose.pose.orientation
        self.robot_theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.odom_received = True

    def on_distance(self, msg):
        self.current_distance = msg.data
        self.get_logger().info(f"Distance: {self.current_distance:.1f}")

    def on_goal(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.goal_received = True
        self.get_logger().info(f"New Goal Received: x={msg.x}, y={msg.y}")

    def control_step(self):
        # Wait until we have both a goal and our own position
        if not self.goal_received or not self.odom_received:
            return

        ex = self.target_x - self.robot_x
        ey = self.target_y - self.robot_y
        dist_to_goal = math.sqrt(ex**2 + ey**2)

        if dist_to_goal < self.goal_tolerance:
            self.cmd_pub.publish(Twist()) # Corrected method name
            self.get_logger().info("Goal Reached!")
            self.goal_received = False # Reset until next goal
            return

        # State Logic
        if self.current_distance < self.distance_threshold:
            self.state = "AVOID_OBSTACLE"
        elif self.state == "AVOID_OBSTACLE" and self.current_distance > (self.distance_threshold + 10.0):
            self.state = "GO_TO_GOAL"

        v = 0.0
        omega = 0.0

        if self.state == "AVOID_OBSTACLE":
            v = 0.0 
            omega = 0.5 
            self.get_logger().warn("Obstacle Detected! Turning...")
        else:
            theta_T = math.atan2(ey, ex)
            e_theta = wrap_to_pi(theta_T - self.robot_theta)
            e_theta_deg = math.degrees(e_theta)
            omega = clip(self.k_theta * e_theta, -self.omega_max, self.omega_max)

            if abs(e_theta) > self.theta_align:
                v = 0.0
                self.get_logger().info(
                    f"ALIGNING: Error is {e_theta_deg:.1f}°. Spinning...", 
                    throttle_duration_sec=0.5
                )
            else:
                # FIXED: changed 'd' to 'dist_to_goal'
                v = clip(self.k_d * dist_to_goal, 0.0, self.v_max)
                self.get_logger().info(
                    f"ALIGNED: Error {e_theta_deg:.1f}°. Driving forward.", 
                    throttle_duration_sec=1.0
                )


            # Smooth slowing down
            slow_radius = 0.4
            if dist_to_goal < slow_radius:
                v *= dist_to_goal / slow_radius

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = GoToBeaconOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
