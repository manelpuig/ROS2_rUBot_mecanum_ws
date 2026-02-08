#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class RobotSelfControl(Node):
    def __init__(self):
        super().__init__('robot_selfcontrol_node')

        self.declare_parameter('distance_laser', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('rotation_speed', 0.3)

        self._distanceLaser = self.get_parameter('distance_laser').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value

        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.angular.z = 0.0

        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data  # IMPORTANT
        )

        # publish cmd_vel continuously
        self.create_timer(0.1, self.publish_cmd_vel)

        self._shutting_down = False
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self._last_info_time = self.start_time

    def publish_cmd_vel(self):
        self._cmdVel.publish(self._msg)

    def laser_callback(self, scan):
        if self._shutting_down:
            return

        angle_min_deg = scan.angle_min * 180.0 / 3.14159
        angle_increment_deg = scan.angle_increment * 180.0 / 3.14159

        custom_range = [
            (distance, i) for i, distance in enumerate(scan.ranges)
            if scan.range_min < distance < scan.range_max
            and -150 <= (angle_min_deg + i * angle_increment_deg) <= 150
        ]

        if not custom_range:
            return

        closest_distance, element_index = min(custom_range)
        angle_closest_deg = angle_min_deg + element_index * angle_increment_deg

        # Determine the zone

        # Obstacle detected
        if -45 <= angle_closest_deg <= 45:
            if closest_distance < self._distanceLaser:
                self._msg.linear.x = 0.0
                self._msg.angular.z = 0.0
            else:
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = 0.0
        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0.0


def main(args=None):
    rclpy.init(args=args)
    robot = RobotSelfControl()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()


if __name__ == '__main__':
    main()
