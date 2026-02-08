#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower_node')

        # -------------------- Parameters --------------------
        self.declare_parameter('distance_limit', 0.5)    # desired right-wall distance
        self.declare_parameter('forward_speed', 0.10)    # max linear speed
        self.declare_parameter('min_speed', 0.03)        # minimum linear speed
        self.declare_parameter('kp_wall', 1.5)           # P gain for wall following
        self.declare_parameter('max_ang', 0.6)           # angular speed limit
        self.declare_parameter('time_to_stop', 30.0)     # auto-stop time

        self.base_distance = float(self.get_parameter('distance_limit').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.kp = float(self.get_parameter('kp_wall').value)
        self.max_ang = float(self.get_parameter('max_ang').value)
        self.time_to_stop = float(self.get_parameter('time_to_stop').value)

        # Last command
        self.cmd = Twist()

        # ROS interfaces
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.cmd_timer = self.create_timer(0.1, self.publish_cmd)   # 10 Hz
        self.info_timer = self.create_timer(1.0, self.log_info)
        self.stop_timer = self.create_timer(0.05, self.stop_watchdog)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.state = "Idle"
        self.last_logged_state = None
        self.shutting_down = False

        self.get_logger().info("Wall follower with P-control started")

    # ------------------------------------------------------
    def stop_watchdog(self):
        if self.shutting_down:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.start_time >= self.time_to_stop:
            self.get_logger().info("Time limit reached — stopping")
            self.stop()

    # ------------------------------------------------------
    def stop(self):
        self.shutting_down = True
        self.cmd = Twist()

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

        for t in [self.cmd_timer, self.info_timer, self.stop_timer]:
            try:
                t.cancel()
            except Exception:
                pass

    # ------------------------------------------------------
    def publish_cmd(self):
        if not self.shutting_down:
            self.publisher.publish(self.cmd)

    # ------------------------------------------------------
    def laser_callback(self, scan):
        if self.shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT, FR_RIGHT, RIGHT = [], [], []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)

        min_front = min(FRONT) if FRONT else float('inf')
        min_fr = min(FR_RIGHT) if FR_RIGHT else float('inf')
        min_right = min(RIGHT) if RIGHT else float('inf')

        twist = Twist()
        action = ""

        # ------------------ SAFETY RULES ------------------
        if min_front < self.base_distance:
            twist.linear.x = 0.0
            twist.angular.z = +self.max_ang
            action = f"FRONT obstacle {min_front:.2f} m → turn LEFT"

        elif min_fr < self.base_distance:
            twist.linear.x = self.min_speed
            twist.angular.z = +self.max_ang
            action = f"FRONT-RIGHT obstacle {min_fr:.2f} m → turn LEFT"

        # ---------------- WALL FOLLOW (P CONTROL) ----------------
        elif math.isfinite(min_right):
            error = min_right - self.base_distance

            # Proportional controller
            ang_z = -self.kp * error
            ang_z = max(-self.max_ang, min(self.max_ang, ang_z))

            # Slow down when turning
            speed_factor = max(0.0, 1.0 - abs(ang_z) / self.max_ang)
            lin_x = self.min_speed + (self.v_lin - self.min_speed) * speed_factor

            twist.linear.x = lin_x
            twist.angular.z = ang_z

            action = (
                f"Wall follow | d={min_right:.2f} m | "
                f"err={error:.2f} | v={lin_x:.2f} | w={ang_z:.2f}"
            )

        else:
            action = "No wall detected → STOP"

        self.cmd = twist

        if action != self.last_logged_state:
            self.get_logger().info(action)
            self.last_logged_state = action

        self.state = action

    # ------------------------------------------------------
    def log_info(self):
        if not self.shutting_down:
            self.get_logger().info(self.state)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
