#!/usr/bin/env python3

import sys
import rclpy
from gazebo_ros_link_attacher.srv import Attach


MAX_RETRIES = 3


def attach(node, client, model1, link1, model2, link2):
    req = Attach.Request()
    req.model_name_1 = model1
    req.link_name_1 = link1
    req.model_name_2 = model2
    req.link_name_2 = link2

    for attempt in range(1, MAX_RETRIES + 1):
        node.get_logger().info(
            f'Attaching {model1}/{link1} <-> {model2}/{link2} (intento {attempt}/{MAX_RETRIES})'
        )
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        if not future.done():
            node.get_logger().error('Timeout esperando respuesta del servicio /attach')
            continue

        result = future.result()
        if result is not None and result.ok:
            node.get_logger().info('Attach OK')
            return True

        node.get_logger().error(f'Attach fallido (resultado: {result})')

    return False


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('demo_attach_links')

    attach_srv = node.create_client(Attach, '/attach')
    while not attach_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Esperando servicio {attach_srv.srv_name}...')

    node.get_logger().info('Conectado al servicio /attach')

    # Unir dron con extremo libre de la cuerda (link_final)
    if not attach(node, attach_srv, 'sjtu_drone', 'base_link', 'tether', 'link_final'):
        node.get_logger().error('No se pudo unir dron-cuerda. Abortando.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Unir winch del UGV con el extremo fijo de la cuerda (link_0)
    if not attach(node, attach_srv, 'rs_robot', 'box_central', 'tether', 'link_0'):
        node.get_logger().error('No se pudo unir winch-cuerda. Abortando.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info('Todos los enganches completados correctamente.')
    node.destroy_node()
    rclpy.shutdown()
