#!/usr/bin/env python3

import os
import rclpy
import pandas as pd
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Imu, BatteryState, Joy, NavSatFix
from geometry_msgs.msg import Twist, Vector3Stamped, QuaternionStamped
from std_msgs.msg import UInt8, Float32
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

class DataLoggerNode(Node):

    def __init__(self, bag_path):
        super().__init__('data_logger_node_theatre')
        self.dir_simulation_data = '/home/upo/marsupial/src/marsupial_simulator_ros2/simulation_data'

        os.makedirs(self.dir_simulation_data, exist_ok=True)

        # Data structures for UGV and UAV
        self.ugv_data = {
            'time': [], 'position_x': [], 'position_y': [], 'position_z': [],
            'velocity_x': [], 'velocity_y': [], 'velocity_z': []
        }
        self.uav_data = {
            'time': [], 'imu_orientation_x': [], 'imu_orientation_y': [], 'imu_orientation_z': [], 'imu_orientation_w': [],
            'imu_angular_velocity_x': [], 'imu_angular_velocity_y': [], 'imu_angular_velocity_z': [],
            'imu_linear_acceleration_x': [], 'imu_linear_acceleration_y': [], 'imu_linear_acceleration_z': [],
            'battery': [], 'gps_latitude': [], 'gps_longitude': []
        }

        self.start_time = None
        self.reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')

        if not os.path.exists(bag_path):
            raise FileNotFoundError(f"El archivo .bag no se encuentra en la ruta especificada: {bag_path}")
        
        self.reader.open(storage_options, converter_options)

    def get_current_time(self, timestamp):
        if self.start_time is None:
            self.start_time = timestamp / 1e9
        return (timestamp / 1e9) - self.start_time

    def ensure_length(self, data_dict):
        """
        Ensure all lists in the dictionary have the same length by appending
        the last known value or 0 if no values have been added yet.
        """
        max_len = max(len(lst) for lst in data_dict.values())
        for key in data_dict:
            while len(data_dict[key]) < max_len:
                if len(data_dict[key]) == 0:
                    data_dict[key].append(0)  
                else:
                    data_dict[key].append(data_dict[key][-1])  

    def process_messages(self):
        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            timestamp = self.get_current_time(t)

            # UGV topics
            if topic == '/arco/idmind_motors/odom':
                msg = deserialize_message(data, Odometry)
                self.ugv_data['time'].append(timestamp)
                self.ugv_data['position_x'].append(msg.pose.pose.position.x)
                self.ugv_data['position_y'].append(msg.pose.pose.position.y)
                self.ugv_data['position_z'].append(msg.pose.pose.position.z)
                self.ugv_data['velocity_x'].append(msg.twist.twist.linear.x)
                self.ugv_data['velocity_y'].append(msg.twist.twist.linear.y)
                self.ugv_data['velocity_z'].append(msg.twist.twist.linear.z)
                self.ensure_length(self.ugv_data)  

            # UAV topics
            elif topic == '/dji_sdk/imu':
                msg = deserialize_message(data, Imu)
                self.uav_data['time'].append(timestamp)
                self.uav_data['imu_orientation_x'].append(msg.orientation.x)
                self.uav_data['imu_orientation_y'].append(msg.orientation.y)
                self.uav_data['imu_orientation_z'].append(msg.orientation.z)
                self.uav_data['imu_orientation_w'].append(msg.orientation.w)
                self.uav_data['imu_angular_velocity_x'].append(msg.angular_velocity.x)
                self.uav_data['imu_angular_velocity_y'].append(msg.angular_velocity.y)
                self.uav_data['imu_angular_velocity_z'].append(msg.angular_velocity.z)
                self.uav_data['imu_linear_acceleration_x'].append(msg.linear_acceleration.x)
                self.uav_data['imu_linear_acceleration_y'].append(msg.linear_acceleration.y)
                self.uav_data['imu_linear_acceleration_z'].append(msg.linear_acceleration.z)
                self.ensure_length(self.uav_data)  

            elif topic == '/dji_sdk/battery_state':
                msg = deserialize_message(data, BatteryState)
                self.uav_data['battery'].append(msg.percentage)
                self.ensure_length(self.uav_data)  

            elif topic == '/dji_sdk/gps_position':
                msg = deserialize_message(data, NavSatFix)
                self.uav_data['gps_latitude'].append(msg.latitude)
                self.uav_data['gps_longitude'].append(msg.longitude)
                self.ensure_length(self.uav_data)  

        self.save_data()

    def save_data(self):
        # Save UGV data
        ugv_df = pd.DataFrame(self.ugv_data)
        ugv_csv_path = os.path.join(self.dir_simulation_data, 'ugv_data_theatre.csv')
        ugv_df.to_csv(ugv_csv_path, index=False)

        # Save UAV data
        uav_df = pd.DataFrame(self.uav_data)
        uav_csv_path = os.path.join(self.dir_simulation_data, 'uav_data_theatre.csv')
        uav_df.to_csv(uav_csv_path, index=False)

        self.get_logger().info(f'Data saved to {ugv_csv_path} and {uav_csv_path}')

def main(args=None):
    rclpy.init(args=args)
    # bag_path = '/home/upo/marsupial/src/marsupial_simulator_ros2/bags/theatre_ros2/theatre_ros2.db3'  
    bag_path = '/home/upo/marsupial/src/marsupial_simulator_ros2/bags/test0/bags_nov_2023/2023-11-03-13-30-54/2023-11-03-13-30-54.db3'  
    data_logger_node = DataLoggerNode(bag_path)

    try:
        data_logger_node.process_messages()
    except KeyboardInterrupt:
        data_logger_node.get_logger().info('Keyboard interrupt detected, saving data...')
        data_logger_node.save_data()
        data_logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
