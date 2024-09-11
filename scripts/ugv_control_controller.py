#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

vel_msg = Twist()  # robot velosity
mode_selection = 0 # 1:in-phase, 2:pivot turn, 3:winch mode,  4:none

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_seperation = 0.122
        self.wheel_base = 0.156
        self.wheel_radius = 0.026
        self.wheel_steering_y_offset = 0.03
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset

        self.pos = np.array([0,0,0,0], float)
        self.vel = np.array([0,0,0,0,0], float) #left_front, right_front, left_rear, right_rear, winch

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global vel_msg, mode_selection, box_central_position

        # in-phase
        if(mode_selection == 1):
            # self.get_logger().info('Mode: In-Phase')
            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)
            
            if(vel_msg.linear.x != 0):
                ang = vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0
            
            self.pos[0] = math.atan(ang)
            self.pos[1] = math.atan(ang)
            self.pos[2] = self.pos[0]
            self.pos[3] = self.pos[1]
            
            self.vel[:] = sign*V
            self.vel[4] = 0
            
        # pivot turn
        elif(mode_selection == 2):
            # self.get_logger().info('Mode: Pivot Turn')
            self.pos[0] = -math.atan(self.wheel_base/self.steering_track)
            self.pos[1] = math.atan(self.wheel_base/self.steering_track)
            self.pos[2] = math.atan(self.wheel_base/self.steering_track)
            self.pos[3] = -math.atan(self.wheel_base/self.steering_track)
            
            self.vel[0] = -vel_msg.angular.z
            self.vel[1] = vel_msg.angular.z
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]
            self.vel[4] = 0

        elif(mode_selection == 3):
            # self.get_logger().info('Mode: Winch')
            self.pos[:] = 0
            self.vel[:] = 0
            self.vel[4] = vel_msg.linear.x*0.3
            
        else:
            # self.get_logger().info('Mode: None')
            self.pos[:] = 0
            self.vel[:] = 0
        
        pos_array = Float64MultiArray(data=self.pos) 
        vel_array = Float64MultiArray(data=self.vel) 
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)

        self.pos[:] = 0
        self.vel[:] = 0

class Joy_subscriber(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, data):
        global vel_msg, mode_selection, box_central_position

        ####### Use with PS4 controller: 
        ####### Press 'X' button to select 'in_phase' and use left joystick to navigate
        ####### Press 'R1' button to select 'pivot' and use right joystick to rotate         
        ####### 
                
        if(data.buttons[0] == 1):   # in-phase # X button of PS4 controller
            mode_selection = 1
        elif(data.buttons[4] == 1): # pivot turn # R1, R2 button of PS4 controller
            mode_selection = 2
        elif(data.buttons[5] == 1): # pivot turn # R1, R2 button of PS4 controller
            mode_selection = 2
        elif(data.buttons[2] == 1): # Triangle to modify winch
            mode_selection = 3
        else:
            mode_selection = 4

        vel_msg.linear.x = data.axes[1]*7.5
        vel_msg.linear.y = data.axes[0]*7.5
        vel_msg.angular.z = data.axes[3]*10
    
if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    joy_subscriber = Joy_subscriber()


    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(joy_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

