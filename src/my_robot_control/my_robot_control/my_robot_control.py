#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_py
import time
import sys

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.robot_x = 0
        self.robot_y = 0
        self.robot_f = 0

        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for command velocity
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Declare parameters
        self.declare_parameter('vx', 0.3)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('w', 0.0)
        self.declare_parameter('td', 3)
        # Get parameters
        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.w = self.get_parameter('w').value
        self.td = self.get_parameter('td').value
        
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        #(_, _, self.robot_f) = tf2_py.transformations.euler_from_quaternion(q)

    def move_robot(self):
        vel = Twist()
        start_time = time.time()
        try:
            while time.time() - start_time < self.td:
                self.get_logger().info('Robot running')
                vel.linear.x = self.vx
                vel.linear.y = self.vy
                vel.angular.z = self.w
                self.publisher.publish(vel)
                #self.get_logger().info('Linear Vel_x = %f, Linear Vel_y = %f, Angular Vel = %f', self.vx, self.vy, self.w)
                self.get_logger().info('Time Duration = '+str(time.time() - start_time))
                time.sleep(0.1)  # Sleep to prevent spamming the loop
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupt received, stopping robot.')
        finally:
            self.get_logger().warn('Stopping robot')
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        controller.move_robot()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().warn('Keyboard interrupt received, shutting down.')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
