#!/usr/bin/env python3
"""
Spawns a box at a random (x,y) on the conveyor and
immediately sets the belt power to a random value ∈ [5, 20].

• Uses the same service you invoke from the CLI:
    /CONVEYORPOWER  →  conveyorbelt_msgs/srv/ConveyorBeltControl
• No custom log tricks, no timer complexities—just a simple while-loop.
"""

import os
import time
import random

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import xacro
from gazebo_msgs.srv import SpawnEntity
from conveyorbelt_msgs.srv import ConveyorBeltControl


class ConveyorSpawner(Node):
    def __init__(self):
        super().__init__('conveyor_spawner')

        # --- service clients -------------------------------------------------
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.conv_cli  = self.create_client(ConveyorBeltControl, '/CONVEYORPOWER')

        self.get_logger().info('Waiting for /spawn_entity …')
        self.spawn_cli.wait_for_service()
        self.get_logger().info('Waiting for /CONVEYORPOWER …')
        self.conv_cli.wait_for_service()
        self.get_logger().info('Both services are ready.')

        # --- load box URDF once ---------------------------------------------
        urdf_path = os.path.join(
            get_package_share_directory('conveyorbelt_gazebo'),
            'urdf', 'box.urdf'
        )
        self.box_xml = xacro.process_file(urdf_path).toxml()

        # --- spawn limits & bookkeeping -------------------------------------
        self.x_min, self.x_max = 0.55, 0.85
        self.y_min, self.y_max = -0.15, 0.13
        self.z_const           = 0.76
        self.count             = 0

    # --------------------------------------------------------------------- #
    def _spawn_one_box(self):
        """Call /spawn_entity exactly like you'd do from the CLI."""
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)

        req = SpawnEntity.Request()
        req.name = f'box_{self.count}'
        req.xml  = self.box_xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = self.z_const

        self.get_logger().info(
            f'[#{self.count}] Spawning at x={x:.2f}, y={y:.2f}'
        )
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        ok = (fut.result() is not None)
        if ok:
            self.get_logger().info('  → spawn OK')
        else:
            self.get_logger().error('  ✗ spawn failed')

    # --------------------------------------------------------------------- #
    def _set_random_power(self):
        """Call /CONVEYORPOWER just like your manual ros2 service call."""
        power = float(random.randint(5, 20))   # never 0
        req = ConveyorBeltControl.Request(power=power)

        self.get_logger().info(f'  Setting belt power → {power}')
        fut = self.conv_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if resp and resp.success:
            self.get_logger().info('  → belt power OK')
        else:
            self.get_logger().error('  ✗ belt power failed')

    # --------------------------------------------------------------------- #
    def loop(self):
        self._spawn_one_box()
        self._set_random_power()
        self.count += 1


def main():
    rclpy.init()
    node = ConveyorSpawner()
    try:
        while rclpy.ok():
            node.loop()
            time.sleep(7.0)               # 9-second interval
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
