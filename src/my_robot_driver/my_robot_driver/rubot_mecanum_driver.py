#!/usr/bin/env python3

import pigpio
import time as t
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler
import threading
import tf2_ros
from .rubot_mecanum_library import Encoder, MPID, DCMotorController

class RubotDrive(Node):
    def __init__(self):
        super().__init__('dc_motor_controller')
        
        # Robot Constant Parameter definition
        self.xn = 0.082 
        self.yn = 0.105 
        self.r = 0.025 # Other robot 0.04
        self.K = abs(self.xn) + abs(self.yn) - 0.02
        self.max_rmp = 330
        self.resolution = 2 * math.pi * self.r / 1320 # 1440
        
        # Odometry Variables
        self.vx = 0
        self.vy = 0
        self.w = 0
        self.theta = 0
        self.x = 0
        self.y = 0
        self.vxi = 0
        self.vyi = 0
        self.wi = 0
        
        # PID
        self.kp = 0.1 #0.1
        self.kd = 0.1 #0.1
        self.ki = 0.4 #0.45
        
        # Encoder position variables 
        self.position1 = 0
        self.position2 = 0
        self.position3 = 0
        self.position4 = 0

        # Initiate pigpio
        self.pi = pigpio.pi()
        
        # Motor and Encoder Pin distribution and configuration
        self.EnA = Encoder(self.pi, 24, 25, self.callback1)
        self.PIDA = MPID(self.kp, self.ki, self.kd, True, self.r, self.resolution, self.max_rmp)
        self.MotorA = DCMotorController(self.pi, 13, 27, 22, 10)
        
        self.EnB = Encoder(self.pi, 23, 15, self.callback2)
        self.PIDB = MPID(self.kp, self.ki, self.kd, False, self.r, self.resolution, self.max_rmp)
        self.MotorB = DCMotorController(self.pi, 19, 4, 17, 10)        

        self.EnC = Encoder(self.pi, 16, 20, self.callback3)
        self.PIDC = MPID(self.kp, self.ki, self.kd, True, self.r, self.resolution, self.max_rmp)
        self.MotorC = DCMotorController(self.pi, 12, 5, 6, 10)
        
        self.EnD = Encoder(self.pi, 26, 21, self.callback4)
        self.PIDD = MPID(self.kp, self.ki, self.kd, True, self.r, self.resolution, self.max_rmp)
        self.MotorD = DCMotorController(self.pi, 18, 11, 9, 10)
        
        # ROS-related variables, msg and node creation
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Twist, '/cmd_vel', self.speed_callback, 10)
        self.create_subscription(Bool, 'reset_odom', self.reset_odom_callback, 10)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Sampling and Time control variables
        self.ctrlrate = 1
        self.lastctrl = self.get_clock().now()
        self.num = 0
        self.sample = 1
        
    def callback1(self, pos):
        self.position1 += pos

    def callback2(self, pos):
        self.position2 += pos

    def callback3(self, pos):
        self.position3 += pos

    def callback4(self, pos):
        self.position4 += pos
        
    def rpm2pwm(self, rpm):
        return int(rpm / self.max_rmp * 255)
    
    def speed2rpm(self, spd):
        return int(30.0 * spd / (self.r * math.pi))
    
    def speed2pwm(self, spd):
        return self.rpm2pwm(self.speed2rpm(spd))
        
    def InverseKinematic(self, vx, vy, omega):
        pwmA = self.speed2pwm((vx - vy - self.K * omega))
        pwmB = self.speed2pwm((vx + vy + self.K * omega))
        pwmC = self.speed2pwm((vx + vy - self.K * omega))
        pwmD = self.speed2pwm((vx - vy + self.K * omega))
        return pwmA, pwmB, pwmC, pwmD

    def ForwardKinematic(self, wA, wB, wC, wD):
        vx = self.r / 4 * (wA + wB + wC + wD)
        vy = self.r / 4 * (-wA + wB + wC - wD)
        omega = self.r / (4 * self.K) * (-wA + wB - wC + wD)
        return vx, vy, omega

    def speed_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.w = msg.angular.z
        self.lastctrl = self.get_clock().now()

    def reset_odom_callback(self, msg):
        if msg.data:
            self.x = 0
            self.y = 0
            self.theta = 0
            self.num = 0

    def shutdown(self):
        self.MotorA.speed(self.pi, 0)
        self.MotorB.speed(self.pi, 0)
        self.MotorC.speed(self.pi, 0)
        self.MotorD.speed(self.pi, 0)
        self.pi.stop()
        self.destroy_node()

    def publish_odom(self, x, y, theta, vx, vy, vth):
        current_time = self.get_clock().now().to_msg()
        
        odom_quat = quaternion_from_euler(0, 0, theta)
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
        
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        self.broadcaster.sendTransform(t)

    def update_odom(self):
        self.current = self.get_clock().now()
        self.delta = (self.current - self.lastctrl).nanoseconds / 1e9
        self.lastctrl = self.current

        self.num += 1
        if self.num == self.sample:
            self.num = 0

            pwmA, pwmB, pwmC, pwmD = self.InverseKinematic(self.vx, self.vy, self.w)

            self.PIDA.update(pwmA, self.position1)
            self.PIDB.update(pwmB, self.position2)
            self.PIDC.update(pwmC, self.position3)
            self.PIDD.update(pwmD, self.position4)

            self.MotorA.speed(self.pi, self.PIDA.output)
            self.MotorB.speed(self.pi, self.PIDB.output)
            self.MotorC.speed(self.pi, self.PIDC.output)
            self.MotorD.speed(self.pi, self.PIDD.output)

            wA = self.PIDA.current_rpm
            wB = self.PIDB.current_rpm
            wC = self.PIDC.current_rpm
            wD = self.PIDD.current_rpm

            self.vxi, self.vyi, self.wi = self.ForwardKinematic(wA, wB, wC, wD)

            self.x += (self.vxi * math.cos(self.theta) - self.vyi * math.sin(self.theta)) * self.delta
            self.y += (self.vxi * math.sin(self.theta) + self.vyi * math.cos(self.theta)) * self.delta
            self.theta += self.wi * self.delta

            self.publish_odom(self.x, self.y, self.theta, self.vxi, self.vyi, self.wi)
            
def main(args=None):
    rclpy.init(args=args)
    rubot_drive = RubotDrive()
    try:
        rclpy.spin(rubot_drive)
    except KeyboardInterrupt:
        rubot_drive.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
