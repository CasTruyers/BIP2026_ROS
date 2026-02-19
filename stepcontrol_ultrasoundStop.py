def control_step(self):
        # For this test, we are completely ignoring self.goal_received and self.odom_received.
        # The robot will just drive the moment the node starts.

        v = 0.0 
        omega = 0.0

        # 1. Check Ultrasound
        # We explicitly ignore -1.0 (and 0.0, just in case). 
        # If the sensor reads a valid number under the threshold, we flag an obstacle.
        is_obstacle_detected = (self.current_distance != -1.0) and (0 < self.current_distance < self.distance_threshold)

        # 2. Simple Drive/Stop Logic
        if is_obstacle_detected:
            # Hit the brakes
            v = 0.0 
            omega = 0.0 
            self.get_logger().warn(
                f"Obstacle Detected at {self.current_distance:.1f}! Stopping...", 
                throttle_duration_sec=1.0
            )
        else:
            # Path is clear (or sensor returned -1). Drive forward at a constant speed.
            # Using self.v_max (0.6) or you can hardcode it to 0.3 for a slower test.
            v = 0.3 
            omega = 0.0
            self.get_logger().info("Path clear. Driving forward.", throttle_duration_sec=1.0)

        # 3. Publish to velocity!
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)
