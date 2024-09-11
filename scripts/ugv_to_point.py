#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Float64
import time

class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')
        self.target_position = Pose()
        self.current_position = Pose()
        self.uav_position = Pose()
        self.target_length = 0.0

        self.cable_length = 0.6    
        self.safety_margin = 0.3
        self.tether_coef = 1.1
        self.kp_winch = 10.0
        self.ki_winch = 0.1
        self.kd_winch = 1.0
        self.velocity_winch_limit = 10
        self.winch_position_x = -0.25
        self.winch_position_z = 0.35

        self.integral_error = 0.0
        self.previous_error = 0.0

        timer_period = 0.02

        self.constant_speed = 1
        self.tolerance = 0.1

        self.pos = np.array([0, 0, 0, 0], float)
        self.vel = np.array([0, 0, 0, 0, 0], float)

        self.last_time = time.time()

        self.pose_subscriber = self.create_subscription(Pose, '/ugv_gt_pose', self.pose_callback, 10)
        self.uav_pose_subscriber = self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.uav_pose_callback, 10)
        self.target_subscriber = self.create_subscription(Pose, '/target_position_ugv', self.target_callback, 10)

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        
        # Publicador para la longitud de la cuerda y la longitud objetivo
        self.pub_cable_length = self.create_publisher(Float64MultiArray, '/cable_length', 10)

        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('UGV controller has been started.')

    def pose_callback(self, msg):
        self.current_position = msg

    def uav_pose_callback(self, msg):
        self.uav_position = msg

    def target_callback(self, msg):
        self.target_position = msg

    def calculate_winch_velocity(self):
        distance = math.sqrt(
            (self.uav_position.position.x - (self.current_position.position.x + self.winch_position_x)) ** 2 +
            (self.uav_position.position.y - self.current_position.position.y) ** 2 +
            (self.uav_position.position.z - (self.current_position.position.z + self.winch_position_z)) ** 2
        )

        self.target_length = distance * self.tether_coef + self.safety_margin
        length_error = self.target_length - self.cable_length

        current_time = time.time()
        dt = current_time - self.last_time

        # PID Control
        self.integral_error += length_error * dt
        derivative_error = (length_error - self.previous_error) / dt

        winch_velocity = (self.kp_winch * length_error +
                          self.ki_winch * self.integral_error +
                          self.kd_winch * derivative_error)

        winch_velocity = max(min(winch_velocity, self.velocity_winch_limit), -self.velocity_winch_limit)

        self.cable_length += winch_velocity * dt 
        self.previous_error = length_error
        self.last_time = current_time

        return winch_velocity, length_error
    
    def control_loop(self):
        if self.current_position is None or self.uav_position is None or self.target_position is None:
            return

        current_time = time.time()
        self.last_time = current_time

        direction_x = self.target_position.position.x - self.current_position.position.x
        direction_y = self.target_position.position.y - self.current_position.position.y

        magnitude = math.sqrt(direction_x**2 + direction_y**2)

        vel_msg = Float64MultiArray()
        pos_msg = Float64MultiArray()
        cable_length_msg = Float64MultiArray()

        if magnitude < self.tolerance:
            control_xy = 0.0
        else:
            sign = np.sign(direction_x)
            control_xy = self.constant_speed * sign

        if direction_x != 0 and magnitude > self.tolerance:
            steering_angle = math.atan(direction_y/direction_x)
        else:
            steering_angle = 0.0

        winch_velocity, length_error = self.calculate_winch_velocity()

        pos_msg.data = [float(steering_angle), float(steering_angle), float(steering_angle), float(steering_angle)]
        vel_msg.data = [float(control_xy), float(control_xy), float(control_xy), float(control_xy), float(winch_velocity)]
        cable_length_msg.data = [float(self.cable_length), float(self.target_length)]

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

        self.pub_cable_length.publish(cable_length_msg)

        self.get_logger().info(f'Distance: {self.target_length:.3f}, Tether length: L={self.cable_length:.3f}, Error={length_error:.3f}, Winch velocity: {winch_velocity:.3f}')

def main(args=None):
    rclpy.init(args=args)
    ugv_controller = UGVController()
    rclpy.spin(ugv_controller)
    ugv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
