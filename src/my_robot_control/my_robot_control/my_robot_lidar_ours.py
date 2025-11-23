import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class RobotForwardStopNode(Node):
    def __init__(self):
        super().__init__('robot_forward_stop_node')
        self.declare_parameter('distance_limit', 0.3)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('time_to_stop', 10.0)

        self._distanceLimit = self.get_parameter('distance_limit').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._time_to_stop = self.get_parameter('time_to_stop').value

        self._msg = Twist()
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self._shutting_down = False
        self._blocked = False  # True if something is detected in front

    def timer_callback(self):
        if self._shutting_down:
            return

        if self._blocked:
            self._msg.linear.x = 0.0
        else:
            self._msg.linear.x = self._forwardSpeed

        self._msg.angular.z = 0.0
        self._cmdVel.publish(self._msg)

        # Optional: stop and shutdown after given time
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.start_time > self._time_to_stop:
            self.get_logger().info("Time up, stopping robot.")
            self.stop()
            self.timer.cancel()
            rclpy.try_shutdown()

    def laser_callback(self, scan):
        # Use the original angle logic (no frame/angle changes)
        angle_min_deg = scan.angle_min * 180.0 / 3.14159
        angle_increment_deg = scan.angle_increment * 180.0 / 3.14159

        min_front_distance = float('inf')

        for i, distance in enumerate(scan.ranges):
            angle_robot_deg = angle_min_deg + i * angle_increment_deg
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue
            # Only consider distances within the front arc [-45, 45] degrees
            if -45 <= angle_robot_deg <= 45:
                if distance < min_front_distance:
                    min_front_distance = distance

        if min_front_distance < self._distanceLimit:
            if not self._blocked:
                self.get_logger().info(
                    f"Obstacle at {min_front_distance:.2f} m. Stopping."
                )
            self._blocked = True
        else:
            if self._blocked:
                self.get_logger().info("Path clear. Moving forward.")
            self._blocked = False

    def stop(self):
        self._shutting_down = True
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self._cmdVel.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = RobotForwardStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
