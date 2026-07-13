#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState
from rclpy.clock import Clock, ClockType

_LATCH_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

class UGVController(Node):

    def __init__(self):
        super().__init__('ugv_controller')
        
        self.target_position = None
        self.current_position = None
        self.uav_position = None
        self.target_uav_position = None
        self.target_length = 0.0
        # True = modo PTR: el winch sigue la longitud planificada del YAML vía /target_length_tether
        # False = modo RTTA: el winch adapta la longitud a distancia_UAV_winch * tether_coef en tiempo real
        self.use_tether_trajectory = False
        self.distance = 0.6
        # Radio del tambor del winch — debe coincidir con box_central en rs_robot.sdf
        # <cylinder><radius>0.1</radius></cylinder>
        # Con radius incorrecto: v_cable_real = ω_cmd × r_sdf ≠ v_deseada
        self.radius = 0.1           # m — radio físico del cilindro del winch
        self.effective_radius = 0.1 # m — mismo radio para integrar la longitud estimada

        # Offset entre sjtu_drone/base_link y el punto físico de enganche de la cuerda
        # (link_final fijado por attach_tether.py). Calculado de las posiciones de spawn:
        #   link_final_world = tether_spawn + drone_point = (-0.25+0.52, -0.02, 0.625-0.16) = (0.27, -0.02, 0.465)
        #   drone_base_link  = (0.2, 0.01, 0.7)
        #   offset = link_final - base_link = (+0.07, -0.03, -0.235)
        self.drone_attachment_x =  0.07   # m
        self.drone_attachment_y = -0.03   # m
        self.drone_attachment_z = -0.235  # m — dominante: cable sale por DEBAJO del drone

        self.tether_length = 1.0
        # Lazo cerrado sobre joint_izquierdo:
        # tether_length = L0 - wind_sign * (theta - theta0) * effective_radius
        # wind_sign = +1 si delta_theta positivo corresponde a enrollar (acortar).
        # Verificar con prueba manual: comanda +ω 1 s y observa si la cuerda se acorta.
        # Si se alarga, invertir a -1.
        self.theta0 = None          # ángulo de referencia capturado al primer /joint_states
        self.current_theta = 0.0
        self.L0 = self.tether_length  # longitud libre inicial (m)
        self.wind_sign = -1  # delta_theta positivo = pagar cuerda (verificado empíricamente)
        self.tether_coef = 1.0  # >1.0 para garantizar slack mínimo y evitar tensión extrema
        # Offset fijo añadido a la longitud calculada (distancia * coef).
        # Valores negativos generan tensión permanente en la cuerda; solo usar
        # en experimentos donde se quiera estudiar la rigidez de la cuerda.
        self.safety_margin = 0.0           

        self.kp_winch = 2.5
        self.ki_winch = 0.0
        self.kd_winch = 0.1
        self.winch_position_x = -0.25
        self.winch_position_z = 0.35

        self.integral_error = 0.0
        self.integral_max = 5.0          # anti-windup
        self.previous_error = 0.0
        self.velocity_winch_limit = 3.14  # rad/s — alineado con límite de ros2_control (rs_robot.sdf)

        timer_period = 0.02

        self.constant_speed = 0.5
        self.tolerance = 0.1

        self.pos = np.array([0, 0, 0, 0], float)
        self.vel = np.array([0, 0, 0, 0, 0], float)

        self.clock = self.get_clock()
        self.last_time = None  # None indica primera iteración

        self.pose_subscriber = self.create_subscription(Pose, '/ugv_gt_pose', self.pose_callback, 10)
        self.uav_pose_subscriber = self.create_subscription(Pose, '/sjtu_drone/gt_pose', self.uav_pose_callback, 10)
        # TRANSIENT_LOCAL: recibe el último waypoint aunque el nodo arranque tarde
        self.target_subscriber = self.create_subscription(Pose, '/target_position_ugv', self.target_callback, _LATCH_QOS)
        self.target_uav_subscriber = self.create_subscription(Pose, '/target_position_uav', self.target_uav_callback, _LATCH_QOS)
        self.target_length_subscriber = self.create_subscription(Float64, '/target_length_tether', self.target_length_callback, _LATCH_QOS)
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.pub_cable_length = self.create_publisher(Float64MultiArray, '/cable_length', 10)

        self.timer = self.create_timer(timer_period, self.control_loop)
        self.get_logger().info('UGV controller has been started.')

    def pose_callback(self, msg):
        self.current_position = msg

    def uav_pose_callback(self, msg):
        self.uav_position = msg

    def target_callback(self, msg):
        self.target_position = msg

    def target_uav_callback(self, msg):
        self.target_uav_position = msg

    def target_length_callback(self, msg):
        if self.use_tether_trajectory and self.target_uav_position is not None and self.target_position is not None:
            self.safety_margin = 0.0
            offset_ugv_z = 0.9
            offset_ugv_x = -0.5
            distance = math.sqrt(
                (self.target_uav_position.position.x - (self.target_position.position.x + self.winch_position_x + offset_ugv_x)) ** 2 +
                (self.target_uav_position.position.y - self.target_position.position.y) ** 2 +
                (self.target_uav_position.position.z - (self.target_position.position.z + self.winch_position_z + offset_ugv_z)) ** 2
            )
            if distance > 1e-3:
                target_length_info = msg.data
                self.tether_coef = target_length_info / distance
                self.get_logger().info(
                    f'[PTR] planned={target_length_info:.3f}m  '
                    f'target_dist={distance:.3f}m  coef={self.tether_coef:.4f}'
                )
            else:
                self.get_logger().warn('UAV-winch distance ~0, skipping tether_coef update')

    def _joint_state_cb(self, msg):
        if 'joint_izquierdo' not in msg.name:
            return
        idx = msg.name.index('joint_izquierdo')
        theta = msg.position[idx]
        if math.isnan(theta):
            return  # joint_state_broadcaster publica NaN (sin state_interface position)
        if self.theta0 is None:
            self.theta0 = theta
            self.L0 = self.tether_length  # hereda lo que el fallback open-loop calculó → sin salto
            self.get_logger().info(
                f'[Winch] Referencia capturada: theta0={theta:.4f} rad, L0={self.L0:.3f} m'
            )
        self.current_theta = theta

    def calculate_winch_velocity(self):
        # Corrección: usar la posición real del punto de enganche (link_final),
        # no el centro de base_link del drone.
        cable_end_x = self.uav_position.position.x + self.drone_attachment_x
        cable_end_y = self.uav_position.position.y + self.drone_attachment_y
        cable_end_z = self.uav_position.position.z + self.drone_attachment_z
        self.distance = math.sqrt(
            (cable_end_x - (self.current_position.position.x + self.winch_position_x)) ** 2 +
            (cable_end_y - self.current_position.position.y) ** 2 +
            (cable_end_z - (self.current_position.position.z + self.winch_position_z)) ** 2
        )
        self.target_length = self.distance * self.tether_coef + self.safety_margin
        length_error = self.target_length - self.tether_length

        secs, nsecs = self.clock.now().seconds_nanoseconds()
        current_time = secs + nsecs * 1e-9

        if self.last_time is None:
            # Primera iteración: inicializar sin integrar ni derivar
            self.last_time = current_time
            self.previous_error = length_error
            return 0.0, length_error

        dt = max(current_time - self.last_time, 1e-3)  # mínimo 1 ms
        self.last_time = current_time

        self.integral_error += length_error * dt
        # Anti-windup
        self.integral_error = max(-self.integral_max, min(self.integral_max, self.integral_error))

        derivative_error = (length_error - self.previous_error) / dt
        self.previous_error = length_error

        winch_velocity_linear = (self.kp_winch * length_error +
                                 self.ki_winch * self.integral_error +
                                 self.kd_winch * derivative_error)

        winch_velocity_angular = winch_velocity_linear / self.radius
        # Saturar velocidad del winch
        winch_velocity_angular = max(-self.velocity_winch_limit,
                                     min(self.velocity_winch_limit, winch_velocity_angular))

        if self.theta0 is not None:
            # Lazo cerrado: longitud estimada desde la rotación real del drum
            delta_theta = self.current_theta - self.theta0
            self.tether_length = self.L0 - self.wind_sign * delta_theta * self.effective_radius
        else:
            # Fallback open-loop hasta recibir el primer /joint_states
            self.tether_length += winch_velocity_angular * self.effective_radius * dt

        # self.get_logger().info(f'Vel linear: {winch_velocity_linear:.5f}, vel angular {winch_velocity_angular:.5f}, dt: {dt:.5f}')

        return winch_velocity_angular, length_error
    
    def control_loop(self):
        if self.current_position is None or self.uav_position is None or self.target_position is None:
            return

        # Vector al target en MARCO MUNDO
        dx_w = self.target_position.position.x - self.current_position.position.x
        dy_w = self.target_position.position.y - self.current_position.position.y

        # Yaw del UGV a partir del cuaternio (convención ZYX). Sin esto, el
        # steering se calcula en marco mundo pero las ruedas aplican el ángulo
        # en marco cuerpo → cuando la cuerda induce rotación del chasis, el
        # UGV deja de ir hacia el target.
        q = self.current_position.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Rotar al MARCO CUERPO del UGV
        cos_y = math.cos(-yaw)
        sin_y = math.sin(-yaw)
        dx_b = cos_y * dx_w - sin_y * dy_w
        dy_b = sin_y * dx_w + cos_y * dy_w

        magnitude = math.sqrt(dx_b * dx_b + dy_b * dy_b)

        vel_msg = Float64MultiArray()
        pos_msg = Float64MultiArray()
        cable_length_msg = Float64MultiArray()

        STEER_LIMIT = 2.1  # rad — límite físico del joint de dirección (rs_robot.sdf)

        if magnitude < self.tolerance:
            control_xy = 0.0
            steering_angle = 0.0
        else:
            theta = math.atan2(dy_b, dx_b)  # dirección al target en marco cuerpo (-π, π]
            if abs(theta) <= STEER_LIMIT:
                # Target alcanzable en modo avance directo
                steering_angle = theta
                control_xy = self.constant_speed
            else:
                # Target en el arco trasero (|theta| > 120°): plegar ángulo π y retroceder.
                # El ángulo plegado |π − |theta|| ≤ 1.04 rad, siempre dentro del límite.
                steering_angle = theta - math.copysign(math.pi, theta)
                control_xy = -self.constant_speed

        winch_velocity, length_error = self.calculate_winch_velocity()

        pos_msg.data = [float(steering_angle), float(steering_angle), float(steering_angle), float(steering_angle)]
        vel_msg.data = [float(control_xy), float(control_xy), float(control_xy), float(control_xy), float(winch_velocity)]
        cable_length_msg.data = [float(self.tether_length), float(self.target_length), float(self.distance)]

        self.pub_pos.publish(pos_msg)
        self.pub_vel.publish(vel_msg)

        self.pub_cable_length.publish(cable_length_msg)

        # self.get_logger().info(f'Distance: {self.target_length:.3f}, Tether length: L={self.tether_length:.3f}, Error={length_error:.3f}, Winch velocity: {winch_velocity:.3f}')

def main(args=None):
    rclpy.init(args=args)
    ugv_controller = UGVController()
    try:
        rclpy.spin(ugv_controller)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    ugv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
