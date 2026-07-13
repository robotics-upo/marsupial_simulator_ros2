#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
import yaml
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Float64MultiArray, String
import math
import sys
import os
from ament_index_python.packages import get_package_share_directory

# TRANSIENT_LOCAL depth=1: equivalente a "latched topic" — los suscriptores que
# arranquen tarde reciben el último valor publicado, y el bag recorder no avisa
# de incompatibilidad de QoS.
_LATCH_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class TrajectoryPublisher(Node):
    def __init__(self, mission_name):
        super().__init__('trajectory_publisher')
        self.ugv_poses = []
        self.uav_poses = []
        self.tether_lengths = []
        self.ugv_target_index = 0
        self.uav_target_index = 0
        self.tether_target_index = 0
        self.current_tether_length = 0.0
        self.winch_target_length   = 0.0  # target dinámico del winch (distancia × coef), de /cable_length[1]
        self.current_ugv_pose = None
        self.current_uav_pose = None

        # Umbrales alineados con los tolerances de los controladores (0.1 m)
        self.uav_distance_threshold = 0.15
        self.ugv_distance_threshold = 0.15
        # C8: umbral activo para la convergencia de la cuerda
        self.tether_length_threshold = 0.5

        package_share_directory = get_package_share_directory('marsupial_simulator_ros2')
        mission_file_path = os.path.join(
            package_share_directory, 'optimized_path', f'{mission_name}.yaml'
        )

        self.load_trajectories(mission_file_path)

        self.ugv_subscriber = self.create_subscription(
            Pose, '/ugv_gt_pose', self.ugv_pose_callback, 10
        )
        self.uav_subscriber = self.create_subscription(
            Pose, '/sjtu_drone/gt_pose', self.uav_pose_callback, 10
        )
        self.tether_length_subscriber = self.create_subscription(
            Float64MultiArray, '/cable_length', self.tether_length_callback, 10
        )

        self.ugv_publisher      = self.create_publisher(Pose,    '/target_position_ugv',   _LATCH_QOS)
        self.uav_publisher      = self.create_publisher(Pose,    '/target_position_uav',   _LATCH_QOS)
        self.tether_publisher   = self.create_publisher(Float64, '/target_length_tether',  _LATCH_QOS)
        self.progress_publisher = self.create_publisher(String,  '/trajectory_progress', 10)

        # Flags por waypoint: si ya se logueó que ese agente llegó al target actual
        self._ugv_logged    = False
        self._uav_logged    = False
        self._tether_logged = False
        # Contador para throttle del log de distancias en consola (cada 20 ticks = 2 s a 10 Hz)
        self._log_tick = 0

        # timer a 1 Hz para republish periódico + timer a 10 Hz para chequeo y progreso
        self.republish_timer = self.create_timer(1.0, self.publish_target_positions)
        self.check_timer = self.create_timer(0.1, self.check_and_update_targets)

        self.publish_target_positions()

    def load_trajectories(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)

        self.ugv_poses = [
            data['marsupial_ugv'][f'poses{i}']['pose']
            for i in range(data['marsupial_ugv']['size'])
        ]
        self.uav_poses = [
            data['marsupial_uav'][f'poses{i}']['pose']
            for i in range(data['marsupial_uav']['size'])
        ]
        self.tether_lengths = [
            data['tether'][f'length{i}']['length']
            for i in range(data['tether']['size'])
        ]

        n_ugv = len(self.ugv_poses)
        n_uav = len(self.uav_poses)
        n_tether = len(self.tether_lengths)
        if not (n_ugv == n_uav == n_tether):
            raise ValueError(
                f'Trayectorias desalineadas: ugv={n_ugv}, uav={n_uav}, tether={n_tether}'
            )

        self.get_logger().info(f'Cargados {n_ugv} waypoints (UGV, UAV, cuerda).')

    # Callbacks solo almacenan; el chequeo lo hace el timer
    def ugv_pose_callback(self, msg):
        self.current_ugv_pose = msg

    def uav_pose_callback(self, msg):
        self.current_uav_pose = msg

    def tether_length_callback(self, msg):
        self.current_tether_length = msg.data[0]
        if len(msg.data) > 1:
            self.winch_target_length = msg.data[1]  # target dinámico (distancia × coef)

    def _ugv_distance(self):
        if self.current_ugv_pose is None:
            return float('inf')
        cp = self.current_ugv_pose.position
        tp = self.ugv_poses[self.ugv_target_index]['position']
        return math.sqrt((cp.x - tp['x'])**2 + (cp.y - tp['y'])**2)

    def _uav_distance(self):
        if self.current_uav_pose is None:
            return float('inf')
        cp = self.current_uav_pose.position
        tp = self.uav_poses[self.uav_target_index]['position']
        return math.sqrt((cp.x - tp['x'])**2 + (cp.y - tp['y'])**2 + (cp.z - tp['z'])**2)

    def _tether_error(self):
        # Modo RTTA: error entre estimación actual y target dinámico del winch (distancia × coef)
        # El YAML se ignora; la cuerda siempre sigue la distancia UAV-UGV
        return abs(self.winch_target_length - self.current_tether_length)

    def check_and_update_targets(self):
        if self.current_ugv_pose is None or self.current_uav_pose is None:
            return

        n_total = len(self.ugv_poses)
        idx     = self.ugv_target_index

        ugv_d  = self._ugv_distance()
        uav_d  = self._uav_distance()
        teth_e = self._tether_error()

        ugv_ok  = ugv_d  < self.ugv_distance_threshold
        uav_ok  = uav_d  < self.uav_distance_threshold
        teth_ok = teth_e < self.tether_length_threshold

        # --- Detección individual: log una sola vez por waypoint cuando cada agente llega ---
        if ugv_ok and not self._ugv_logged:
            self.get_logger().info(
                f'\033[96m[UGV OK] WP {idx+1}/{n_total} — dist={ugv_d:.3f}m '
                f'(esperando UAV={uav_d:.3f}m, Tether err={teth_e:.3f}m)\033[0m'
            )
            self._ugv_logged = True

        if uav_ok and not self._uav_logged:
            self.get_logger().info(
                f'\033[96m[UAV OK] WP {idx+1}/{n_total} — dist={uav_d:.3f}m '
                f'(esperando UGV={ugv_d:.3f}m, Tether err={teth_e:.3f}m)\033[0m'
            )
            self._uav_logged = True

        if teth_ok and not self._tether_logged:
            self.get_logger().info(
                f'\033[96m[TETHER OK] WP {idx+1}/{n_total} — err={teth_e:.3f}m '
                f'(cur={self.current_tether_length:.3f} tgt={self.tether_lengths[idx]:.3f}m, '
                f'esperando UGV={ugv_d:.3f}m UAV={uav_d:.3f}m)\033[0m'
            )
            self._tether_logged = True

        # --- Log throttled en consola cada 2 s (20 ticks a 10 Hz) ---
        self._log_tick += 1
        if self._log_tick >= 20:
            self._log_tick = 0
            self.get_logger().info(
                f'WP {idx+1}/{n_total} | '
                f'UGV {"OK" if ugv_ok else f"{ugv_d:.2f}m"} | '
                f'UAV {"OK" if uav_ok else f"{uav_d:.2f}m"} | '
                f'Tether {"OK" if teth_ok else f"err={teth_e:.2f}m"} '
                f'(cur={self.current_tether_length:.2f} winch_tgt={self.winch_target_length:.2f})'
            )

        # --- Publicar progreso en /trajectory_progress (10 Hz) ---
        progress_msg = String()
        progress_msg.data = (
            f'WP {idx+1}/{n_total} | '
            f'UGV [{"OK" if ugv_ok else f"{ugv_d:.3f}m"}] | '
            f'UAV [{"OK" if uav_ok else f"{uav_d:.3f}m"}] | '
            f'Tether [{"OK" if teth_ok else f"err={teth_e:.3f}m"}] '
            f'cur={self.current_tether_length:.3f} winch_tgt={self.winch_target_length:.3f}'
        )
        self.progress_publisher.publish(progress_msg)

        # --- Avanzar waypoint cuando los tres están OK ---
        if ugv_ok and uav_ok and teth_ok:
            self.get_logger().info(
                f'\033[92m>>> WP {idx+1}/{n_total} COMPLETADO '
                f'[UGV {ugv_d:.3f}m | UAV {uav_d:.3f}m | Tether err={teth_e:.3f}m]\033[0m'
            )
            self.ugv_target_index    += 1
            self.uav_target_index    += 1
            self.tether_target_index += 1

            # Resetear flags para el nuevo waypoint
            self._ugv_logged    = False
            self._uav_logged    = False
            self._tether_logged = False
            self._log_tick      = 0

            if self.ugv_target_index >= n_total:
                self.get_logger().info('\033[92m=== TRAYECTORIA FINALIZADA ===\033[0m')
                self.republish_timer.cancel()
                self.check_timer.cancel()
                rclpy.shutdown()
                return

            self.publish_target_position('ugv')
            self.publish_target_position('uav')
            self.publish_target_position('tether')

            next_idx = self.ugv_target_index
            self.get_logger().info(
                f'Siguiente WP {next_idx+1}/{n_total}: '
                f'UGV({self.ugv_poses[next_idx]["position"]["x"]:.2f},'
                f'{self.ugv_poses[next_idx]["position"]["y"]:.2f}) | '
                f'UAV({self.uav_poses[next_idx]["position"]["x"]:.2f},'
                f'{self.uav_poses[next_idx]["position"]["y"]:.2f},'
                f'{self.uav_poses[next_idx]["position"]["z"]:.2f}) | '
                f'Tether={self.tether_lengths[next_idx]:.3f}m'
            )

    def is_near_target_uav(self, current_pose, target_pose):
        cp = current_pose.position
        tp = target_pose['position']
        distance = math.sqrt(
            (cp.x - tp['x'])**2 + (cp.y - tp['y'])**2 + (cp.z - tp['z'])**2
        )
        return distance < self.uav_distance_threshold

    def is_near_target_ugv(self, current_pose, target_pose):
        cp = current_pose.position
        tp = target_pose['position']
        distance = math.sqrt((cp.x - tp['x'])**2 + (cp.y - tp['y'])**2)
        return distance < self.ugv_distance_threshold

    def is_near_target_tether(self, current_length, target_length):
        return abs(target_length - current_length) < self.tether_length_threshold

    def publish_target_positions(self):
        self.publish_target_position('ugv')
        self.publish_target_position('uav')
        self.publish_target_position('tether')

    def publish_target_position(self, vehicle_type):
        if vehicle_type == 'ugv' and self.ugv_target_index < len(self.ugv_poses):
            pose_data = self.ugv_poses[self.ugv_target_index]
            pose_msg = Pose()
            pose_msg.position.x = float(pose_data['position']['x'])
            pose_msg.position.y = float(pose_data['position']['y'])
            pose_msg.position.z = float(pose_data['position']['z'])
            pose_msg.orientation.w = 1.0
            self.ugv_publisher.publish(pose_msg)

        elif vehicle_type == 'uav' and self.uav_target_index < len(self.uav_poses):
            pose_data = self.uav_poses[self.uav_target_index]
            pose_msg = Pose()
            pose_msg.position.x = float(pose_data['position']['x'])
            pose_msg.position.y = float(pose_data['position']['y'])
            pose_msg.position.z = float(pose_data['position']['z'])
            pose_msg.orientation.w = 1.0
            self.uav_publisher.publish(pose_msg)

        elif vehicle_type == 'tether' and self.tether_target_index < len(self.tether_lengths):
            tether_msg = Float64()
            tether_msg.data = float(self.tether_lengths[self.tether_target_index])
            self.tether_publisher.publish(tether_msg)


def main(args=None):
    rclpy.init(args=args)
    mission_name = sys.argv[1] if len(sys.argv) > 1 else 'test1'
    node = TrajectoryPublisher(mission_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
