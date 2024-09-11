#!/usr/bin/env python3

import os
import rclpy
import pandas as pd
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import PoseStamped

class DataLoggerNode(Node):

    def __init__(self, bag_path):
        super().__init__('data_logger_node')
        self.dir_simulation_data = '/home/upo/marsupial/src/marsupial_simulator_ros2/simulation_data'
        
        os.makedirs(self.dir_simulation_data, exist_ok=True)

        self.drone_data = {'time': [], 'position_x': [], 'position_y': [], 'position_z': [], 'velocity_x': [], 'velocity_y': [], 'velocity_z': [], 'target_position_x': [], 'target_position_y': [], 'target_position_z': []}
        self.ugv_data = {'time': [], 'position_x': [], 'position_y': [], 'position_z': [], 'velocity_x': [], 'velocity_y': [], 'velocity_z': [], 'winch_velocity': [], 'cable_length': [], 'target_length': [], 'distance': [], 'target_position_x': [], 'target_position_y': [], 'target_position_z': []}
        self.tether_data = {'time': []}
        self.current_row_index = -1
        self.current_valid_timestamp = None

        self.start_time = None

        self.collecting_data = True

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

    def process_messages(self):
        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            timestamp = self.get_current_time(t)

            if topic == '/sjtu_drone/gt_pose':
                msg = deserialize_message(data, Pose)
                self.drone_data['time'].append(timestamp)
                self.drone_data['position_x'].append(msg.position.x)
                self.drone_data['position_y'].append(msg.position.y)
                self.drone_data['position_z'].append(msg.position.z)
                self.ensure_length(self.drone_data)

            elif topic == '/sjtu_drone/cmd_vel':
                msg = deserialize_message(data, Twist)
                self.drone_data['velocity_x'].append(msg.linear.x)
                self.drone_data['velocity_y'].append(msg.linear.y)
                self.drone_data['velocity_z'].append(msg.linear.z)
                self.ensure_length(self.drone_data)

            elif topic == '/ugv_gt_pose':
                msg = deserialize_message(data, Pose)
                self.ugv_data['time'].append(timestamp)
                self.ugv_data['position_x'].append(msg.position.x)
                self.ugv_data['position_y'].append(msg.position.y)
                self.ugv_data['position_z'].append(msg.position.z)
                self.ensure_length(self.ugv_data)

            elif topic == '/forward_velocity_controller/commands':
                msg = deserialize_message(data, Float64MultiArray)
                if len(msg.data) >= 5:
                    self.ugv_data['velocity_x'].append(msg.data[0])
                    self.ugv_data['velocity_y'].append(msg.data[1])
                    self.ugv_data['velocity_z'].append(msg.data[2])
                    self.ugv_data['winch_velocity'].append(msg.data[4])
                    self.ensure_length(self.ugv_data)

            elif topic == '/cable_length':
                msg = deserialize_message(data, Float64MultiArray)
                self.ugv_data['cable_length'].append(msg.data[0])
                self.ugv_data['target_length'].append(msg.data[1])
                self.ugv_data['distance'].append(msg.data[2])
                self.ensure_length(self.ugv_data)

            elif topic == '/target_position_uav':
                msg = deserialize_message(data, Pose)
                self.drone_data['target_position_x'].append(msg.position.x)
                self.drone_data['target_position_y'].append(msg.position.y)
                self.drone_data['target_position_z'].append(msg.position.z)
                self.ensure_length(self.drone_data)

            elif topic == '/target_position_ugv':
                msg = deserialize_message(data, Pose)
                self.ugv_data['target_position_x'].append(msg.position.x)
                self.ugv_data['target_position_y'].append(msg.position.y)
                self.ugv_data['target_position_z'].append(msg.position.z)
                self.ensure_length(self.ugv_data)

            elif topic == '/tether_positions':
                msg = deserialize_message(data, PoseStamped)
                frame_id = msg.header.frame_id
                position_x = msg.pose.position.x
                position_y = msg.pose.position.y
                position_z = msg.pose.position.z

                if frame_id == "link_0":
                    self.current_valid_timestamp = timestamp
                    self.tether_data['time'].append(self.current_valid_timestamp)
                    self.current_row_index += 1

                    for key in self.tether_data.keys():
                        if key != 'time':
                            self.tether_data[key].append(None)

                if f'{frame_id}_x' not in self.tether_data:
                    self.tether_data[f'{frame_id}_x'] = [None] * (self.current_row_index + 1)
                    self.tether_data[f'{frame_id}_y'] = [None] * (self.current_row_index + 1)
                    self.tether_data[f'{frame_id}_z'] = [None] * (self.current_row_index + 1)

                self.tether_data[f'{frame_id}_x'][self.current_row_index] = position_x
                self.tether_data[f'{frame_id}_y'][self.current_row_index] = position_y
                self.tether_data[f'{frame_id}_z'][self.current_row_index] = position_z

        self.save_data()

    def ensure_length(self, data_dict):
        max_len = max(len(lst) for lst in data_dict.values())
        for key in data_dict:
            while len(data_dict[key]) < max_len:
                if len(data_dict[key]) == 0:
                    data_dict[key].append(0)
                else:
                    data_dict[key].append(data_dict[key][-1])

    def save_data(self):
        drone_df = pd.DataFrame(self.drone_data)
        ugv_df = pd.DataFrame(self.ugv_data)
        tether_df = pd.DataFrame(self.tether_data)

        drone_df.to_csv(os.path.join(self.dir_simulation_data, 'drone_data.csv'), index=False)
        ugv_df.to_csv(os.path.join(self.dir_simulation_data, 'ugv_data.csv'), index=False)
        tether_df.to_csv(os.path.join(self.dir_simulation_data, 'tether_data.csv'), index=False)
        
        self.get_logger().info('Data has been saved to CSV files.')

def main(args=None):
    rclpy.init(args=args)
    bag_path = '/home/upo/marsupial/src/marsupial_simulator_ros2/bags/test5/test5.db3'  
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
