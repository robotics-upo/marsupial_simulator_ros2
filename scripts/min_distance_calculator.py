#!/usr/bin/env python3
"""
Min Distance Calculator Node

This ROS 2 node subscribes to LiDAR PointCloud2 topics and corresponding Pose topics
for both a Drone (UAV) and a Ground Vehicle (UGV). It filters out points beyond a specified
range (15 meters), transforms the points to the global frame based on the vehicle poses,
excludes points belonging to the UAV, UGV, tether, and ground, calculates the minimum
distance from each vehicle to the nearest obstacle detected by their respective LiDARs,
and publishes the combined filtered point cloud for visualization in RViz. Additionally,
it calculates the minimum distance from the tether to obstacles, excluding tether points
that are near the UAV or UGV.

Author: [Your Name]
Date: [Date]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs_py import point_cloud2
import numpy as np
from scipy.spatial import KDTree
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Header


class MinDistanceCalculator(Node):
    def __init__(self):
        super().__init__('min_distance_calculator')

        # Parametersground_threshold (meters)
        self.timer_period = 1.0  # Timer period in seconds

        # Filtering Parameters
        self.uav_exclusion_radius = 0.7         # Radius around UAV to exclude points (meters)
        self.ugv_exclusion_radius = 1.0         # Radius around UGV to exclude points (meters)
        self.tether_exclusion_distance = 0.3     # Exclusion distance from tether points to obstacles (meters)
        self.ground_threshold = 0.0              # Z-axis threshold to exclude ground points (meters)

        # Storage for the latest messages
        self.drone_point_cloud = None
        self.ugv_point_cloud = None
        self.drone_pose = None
        self.ugv_pose = None
        self.tether_points = []  # List to store tether points

        # Define QoS profile for subscriptions
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Subscribers for LiDAR data
        self.drone_lidar_sub = self.create_subscription(
            PointCloud2,
            '/sjtu_drone/velodyne_plugin/out',
            self.drone_lidar_callback,
            qos_profile
        )
        self.ugv_lidar_sub = self.create_subscription(
            PointCloud2,
            '/rs_robot/velodyne_plugin/out',
            self.ugv_lidar_callback,
            qos_profile
        )

        # Subscribers for pose data
        self.drone_pose_sub = self.create_subscription(
            Pose,
            '/sjtu_drone/gt_pose',
            self.drone_pose_callback,
            qos_profile
        )
        self.ugv_pose_sub = self.create_subscription(
            Pose,
            '/ugv_gt_pose',
            self.ugv_pose_callback,
            qos_profile
        )

        # Subscriber for tether positions as PoseStamped
        self.tether_positions_sub = self.create_subscription(
            PoseStamped,  # Updated to PoseStamped
            '/tether_positions',
            self.tether_positions_callback,
            qos_profile
        )

        # Publisher for the combined filtered point cloud
        self.combined_pcd_pub = self.create_publisher(
            PointCloud2,
            '/combined_filtered_point_cloud',
            qos_profile
        )

        # Timer to trigger distance calculations
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Log initialization
        self.get_logger().info('MinDistanceCalculator node has been initialized and started.')

    def drone_lidar_callback(self, msg):
        """Callback to store the latest Drone LiDAR PointCloud2 message."""
        self.drone_point_cloud = msg
        self.get_logger().debug('Received Drone LiDAR data.')

    def ugv_lidar_callback(self, msg):
        """Callback to store the latest UGV LiDAR PointCloud2 message."""
        self.ugv_point_cloud = msg
        self.get_logger().debug('Received UGV LiDAR data.')

    def drone_pose_callback(self, msg):
        """Callback to store the latest Drone pose."""
        self.drone_pose = msg
        self.get_logger().debug('Received Drone pose data.')

    def ugv_pose_callback(self, msg):
        """Callback to store the latest UGV pose."""
        self.ugv_pose = msg
        self.get_logger().debug('Received UGV pose data.')

    def tether_positions_callback(self, msg):
        """
        Callback to store tether points.
        Handles PoseStamped messages where each pose represents a point along the tether.
        """
        tether_point = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float32)
        self.tether_points.append(tether_point)
        self.get_logger().debug(f'Received tether point: {tether_point}')

    def timer_callback(self):
        """
        Timer callback to process data and calculate minimum distances.
        Executes periodically based on the timer_period.
        """
        # Verify availability of necessary data
        if not all([self.drone_point_cloud, self.ugv_point_cloud, self.drone_pose, self.ugv_pose]):
            self.get_logger().warn('Awaiting all necessary data: LiDAR and pose information for Drone and UGV.')
            return

        # Process Drone LiDAR data
        drone_points = self.extract_and_transform_points(
            self.drone_point_cloud, self.drone_pose
        )
        min_distance_drone = self.calculate_min_distance(drone_points)

        # Log number of Drone points after filtering
        self.get_logger().debug(f'Drone points after filtering: {drone_points.shape[0]}')

        # Process UGV LiDAR data
        ugv_points = self.extract_and_transform_points(
            self.ugv_point_cloud, self.ugv_pose
        )
        min_distance_ugv = self.calculate_min_distance(ugv_points)

        # Log number of UGV points after filtering
        self.get_logger().debug(f'UGV points after filtering: {ugv_points.shape[0]}')

        # Combine the filtered point clouds for visualization
        if drone_points.size > 0 and ugv_points.size > 0:
            combined_points = np.vstack((drone_points, ugv_points))
        elif drone_points.size > 0:
            combined_points = drone_points
        elif ugv_points.size > 0:
            combined_points = ugv_points
        else:
            combined_points = np.array([])

        # Store combined_points as a class attribute for tether distance calculation
        self.combined_points = combined_points

        # Log number of combined points
        self.get_logger().debug(f'Combined points for visualization: {combined_points.shape[0]}')

        # Publish the combined filtered point cloud if available
        if combined_points.size > 0:
            combined_pcd_msg = self.create_point_cloud_msg(combined_points)
            self.combined_pcd_pub.publish(combined_pcd_msg)
            self.get_logger().debug('Published combined filtered point cloud for RViz visualization.')

        # Calculate the minimum distance from tether to obstacles
        min_distance_tether = self.calculate_min_distance_tether_to_obstacles()

        # Construct the minimum distance message
        distance_message = "Minimum Distance - "

        # Drone distance
        if min_distance_drone != np.inf:
            distance_message += f"Drone: {min_distance_drone:.2f} m, "
        else:
            distance_message += "Drone: No data, "

        # UGV distance
        if min_distance_ugv != np.inf:
            distance_message += f"UGV: {min_distance_ugv:.2f} m, "
        else:
            distance_message += "UGV: No data, "

        # Tether distance
        if min_distance_tether is None:
            distance_message += "Tether: No data"
        elif min_distance_tether == np.inf:
            distance_message += "Tether: inf m"
        else:
            distance_message += f"Tether: {min_distance_tether:.2f} m"

        # Log the minimum distance information
        self.get_logger().info(distance_message)

        # Clear tether points after processing to prevent data accumulation
        self.tether_points = []

    def extract_and_transform_points(self, point_cloud, pose):
        """
        Extracts, filters, and transforms points from a PointCloud2 message based on the specified range and pose.
        Additionally excludes points belonging to UAV, UGV, tether, and ground.
        
        Args:
            point_cloud (PointCloud2): The incoming PointCloud2 message.
            pose (Pose): The pose of the vehicle (Drone or UGV).

        Returns:
            np.ndarray: An Nx3 array of filtered and transformed point coordinates in the global frame.
        """
        # Convert PointCloud2 message to a generator of points
        points_gen = point_cloud2.read_points(
            point_cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        # Convert generator to a NumPy array with shape (N, 3)
        points = np.array([[p[0], p[1], p[2]] for p in points_gen], dtype=np.float32)

        if points.size == 0:
            self.get_logger().warn('Received an empty point cloud.')
            return np.array([])

        # Calculate distances from the LiDAR origin
        distances = np.linalg.norm(points, axis=1)

        # Filter out points beyond the specified range
        within_range = distances <= self.lidar_range
        filtered_points = points[within_range]

        if filtered_points.size == 0:
            self.get_logger().warn('No points within the specified LiDAR range.')
            return np.array([])

        # Transform points to the global frame based on the vehicle's pose
        transformed_points = self.transform_points(filtered_points, pose)

        # Apply additional filtering to exclude UAV, UGV, tether, and ground points
        transformed_points = self.additional_filtering(transformed_points)

        return transformed_points

    def transform_points(self, points, pose):
        """
        Transforms point coordinates from the vehicle's frame to the global frame using the vehicle's pose.
        
        Args:
            points (np.ndarray): An Nx3 array of point coordinates in the vehicle's frame.
            pose (Pose): The pose of the vehicle.

        Returns:
            np.ndarray: An Nx3 array of point coordinates in the global frame.
        """
        # Extract translation vector
        translation = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ], dtype=np.float32)

        # Extract rotation matrix from quaternion
        q = pose.orientation
        rotation_matrix = self.quaternion_to_rotation_matrix(q)

        # Apply rotation and translation to transform points to the global frame
        transformed_points = np.dot(points, rotation_matrix.T) + translation

        return transformed_points

    def quaternion_to_rotation_matrix(self, q):
        """
        Converts a quaternion into a rotation matrix.
        
        Args:
            q (geometry_msgs.msg.Quaternion): The quaternion representing rotation.

        Returns:
            np.ndarray: A 3x3 rotation matrix.
        """
        # Normalize the quaternion
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        qx = q.x / norm
        qy = q.y / norm
        qz = q.z / norm
        qw = q.w / norm

        # Compute rotation matrix elements
        r00 = 1 - 2*(qy**2 + qz**2)
        r01 = 2*(qx*qy - qz*qw)
        r02 = 2*(qx*qz + qy*qw)

        r10 = 2*(qx*qy + qz*qw)
        r11 = 1 - 2*(qx**2 + qz**2)
        r12 = 2*(qy*qz - qx*qw)

        r20 = 2*(qx*qz - qy*qw)
        r21 = 2*(qy*qz + qx*qw)
        r22 = 1 - 2*(qx**2 + qy**2)

        rotation_matrix = np.array([
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22]
        ], dtype=np.float32)

        return rotation_matrix

    def additional_filtering(self, points):
        """
        Applies additional filtering to exclude points belonging to UAV, UGV, tether, and ground.
        
        Args:
            points (np.ndarray): An Nx3 array of point coordinates in the global frame.

        Returns:
            np.ndarray: An Nx3 array of filtered point coordinates.
        """
        # Exclude ground points based on Z-axis threshold
        ground_filtered = points[points[:, 2] >= self.ground_threshold]

        # Exclude points within the UAV exclusion radius
        if self.drone_pose is not None:
            uav_position = np.array([
                self.drone_pose.position.x,
                self.drone_pose.position.y,
                self.drone_pose.position.z
            ], dtype=np.float32)
            distances_to_uav = np.linalg.norm(ground_filtered - uav_position, axis=1)
            uav_filtered = ground_filtered[distances_to_uav > self.uav_exclusion_radius]
        else:
            uav_filtered = ground_filtered

        # Exclude points within the UGV exclusion radius
        if self.ugv_pose is not None:
            ugv_position = np.array([
                self.ugv_pose.position.x,
                self.ugv_pose.position.y,
                self.ugv_pose.position.z
            ], dtype=np.float32)
            distances_to_ugv = np.linalg.norm(uav_filtered - ugv_position, axis=1)
            ugv_filtered = uav_filtered[distances_to_ugv > self.ugv_exclusion_radius]
        else:
            ugv_filtered = uav_filtered

        # Exclude points within the tether exclusion distance from any tether point
        if len(self.tether_points) > 0:
            tether_tree = KDTree(self.tether_points)
            distances_to_tether, _ = tether_tree.query(ugv_filtered, distance_upper_bound=self.tether_exclusion_distance)
            # Exclude points within the exclusion distance
            tether_filtered = ugv_filtered[distances_to_tether > self.tether_exclusion_distance]
        else:
            tether_filtered = ugv_filtered

        return tether_filtered

    def calculate_min_distance(self, points):
        """
        Calculates the minimum distance from the vehicle (assumed at origin) to the nearest obstacle.
        
        Args:
            points (np.ndarray): An Nx3 array of point coordinates in the global frame.

        Returns:
            float: The minimum distance in meters. Returns np.inf if no points are available.
        """
        if points.size == 0:
            return np.inf

        # Build a KDTree for efficient nearest neighbor search
        tree = KDTree(points)

        # Query the nearest neighbor to the origin (vehicle's position)
        distance, _ = tree.query([0, 0, 0], k=1)

        return distance

    def calculate_min_distance_tether_to_obstacles(self):
        """
        Calculates the minimum distance from the tether points to the obstacles.
        Excludes tether points that are near the UAV or UGV.
        
        Returns:
            float: The minimum distance in meters. Returns np.inf if no tether points are available or no obstacles detected.
        """
        if not self.tether_points:
            return None  # Indicates that no tether data is available

        # Convert tether points list to a NumPy array
        tether_points_np = np.array(self.tether_points, dtype=np.float32)

        # Filter tether points that are near the UAV or UGV
        tether_filtered = self.filter_tether_points(tether_points_np)

        if tether_filtered.size == 0:
            return np.inf  # All tether points were excluded

        # Ensure that there are obstacles to calculate distances
        if hasattr(self, 'combined_points') and self.combined_points.size > 0:
            obstacles = self.combined_points
        else:
            return np.inf  # No obstacles available for distance calculation

        # Build a KDTree for obstacles
        obstacle_tree = KDTree(obstacles)

        # Query the nearest obstacle for each tether point
        distances, _ = obstacle_tree.query(tether_filtered, k=1)

        if distances.size == 0:
            return np.inf

        # Return the smallest distance found
        min_distance = np.min(distances)

        return min_distance

    def filter_tether_points(self, tether_points):
        """
        Filters out tether points that are within exclusion radii of the UAV or UGV.
        
        Args:
            tether_points (np.ndarray): An Nx3 array of tether point coordinates in the global frame.

        Returns:
            np.ndarray: An Nx3 array of filtered tether points.
        """
        # Exclude tether points near the UAV
        if self.drone_pose is not None:
            uav_position = np.array([
                self.drone_pose.position.x,
                self.drone_pose.position.y,
                self.drone_pose.position.z
            ], dtype=np.float32)
            distances_to_uav = np.linalg.norm(tether_points - uav_position, axis=1)
            tether_filtered = tether_points[distances_to_uav > self.uav_exclusion_radius]
        else:
            tether_filtered = tether_points

        # Exclude tether points near the UGV
        if self.ugv_pose is not None:
            ugv_position = np.array([
                self.ugv_pose.position.x,
                self.ugv_pose.position.y,
                self.ugv_pose.position.z
            ], dtype=np.float32)
            distances_to_ugv = np.linalg.norm(tether_filtered - ugv_position, axis=1)
            tether_filtered = tether_filtered[distances_to_ugv > self.ugv_exclusion_radius]
        else:
            tether_filtered = tether_filtered

        return tether_filtered

    def create_point_cloud_msg(self, points):
        """
        Creates a PointCloud2 message from an array of points.
        
        Args:
            points (np.ndarray): An Nx3 array of point coordinates in the global frame.

        Returns:
            PointCloud2: The constructed PointCloud2 message.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Assumes 'map' is the global frame

        # Create PointCloud2 message with XYZ fields
        pcd_msg = point_cloud2.create_cloud_xyz32(header, points.tolist())

        return pcd_msg


def main(args=None):
    """Main function to initialize and spin the ROS 2 node."""
    rclpy.init(args=args)
    node = MinDistanceCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
