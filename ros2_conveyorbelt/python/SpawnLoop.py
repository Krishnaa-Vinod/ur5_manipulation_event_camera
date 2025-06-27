#!/usr/bin/python3


import os
import time
import xacro
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


def main():
    rclpy.init()
    node = rclpy.create_node('entity_spawner_loop')

    # Create client for service
    client = node.create_client(SpawnEntity, '/spawn_entity')
    node.get_logger().info('Waiting for `/spawn_entity` service...')
    client.wait_for_service()
    node.get_logger().info('Connected to `/spawn_entity`')

    # Hard-coded package and URDF
    package_name = 'conveyorbelt_gazebo'
    urdf_filename = 'box.urdf'
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_filename
    )

    # Process XACRO/URDF once
    doc = xacro.process_file(urdf_path)
    xml = doc.toxml()

    # Positions to spawn (x, y, z)
    spawn_position = (0.0, -0.5, 0.76)
    spawn_interval = 5.0  # seconds between spawns

    count = 0
    try:
        while rclpy.ok():
            request = SpawnEntity.Request()
            request.name = f'box_{count}'
            request.xml = xml
            request.initial_pose.position.x = float(spawn_position[0])
            request.initial_pose.position.y = float(spawn_position[1])
            request.initial_pose.position.z = float(spawn_position[2])

            node.get_logger().info(f'Spawning `{request.name}` at '
                                   f'{spawn_position[0]}, {spawn_position[1]}, {spawn_position[2]}')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

            if future.result() is not None:
                node.get_logger().info(f'Successfully spawned `{request.name}`')
            else:
                node.get_logger().error(
                    f'Failed to spawn `{request.name}`: {future.exception()}')

            count += 1
            time.sleep(spawn_interval)

    except KeyboardInterrupt:
        node.get_logger().info('Spawner loop interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
