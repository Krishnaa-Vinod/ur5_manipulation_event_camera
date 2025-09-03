#!/usr/bin/python3
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  Random Box Spawner - Spawns colored boxes randomly in Gazebo                        #
#  Modified from IFRA-Cranfield SpawnObject.py                                         #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

import argparse
import os
import random
import time
import threading
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
# Import your custom BoxInfo message
from conveyorbelt_gazebo.msg import BoxInfo  # Updated to use custom message
import rclpy
from rclpy.node import Node

class RandomBoxSpawner(Node):
    
    def __init__(self):
        super().__init__('random_box_spawner')
        
        # Publisher for box info with timestamp header
        self.box_info_publisher = self.create_publisher(BoxInfo, '/box_info', 10)
        
        # Box configurations
        self.box_colors = ['black', 'red', 'blue', 'orange', 'green']
        self.spawned_boxes = []
        
        # Spawn boundaries (you can modify these)
        self.x_min = -1.0
        self.x_max = 1.0
        self.y_min = -1.0
        self.y_max = 1.0
        self.z_fixed = 0.76  # Fixed Z position
        
        # Create spawn and delete service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Start keyboard listener thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        self.get_logger().info('Random Box Spawner initialized')
        self.get_logger().info(f'Available colors: {self.box_colors}')
        self.get_logger().info(f'Spawn area: X[{self.x_min}, {self.x_max}], Y[{self.y_min}, {self.y_max}], Z={self.z_fixed}')
        self.get_logger().info('Press "d" to delete all spawned boxes')
    
    def keyboard_listener(self):
        """Listen for keyboard input to delete boxes"""
        try:
            while self.running:
                user_input = input().strip().lower()
                if user_input == 'd':
                    self.delete_all_boxes()
                elif user_input == 'q':
                    self.get_logger().info('Quit command received')
                    self.running = False
                    break
        except EOFError:
            pass
        except KeyboardInterrupt:
            self.running = False
    
    def wait_for_services(self):
        """Wait for spawn and delete services to be available"""
        self.get_logger().info('Waiting for /spawn_entity service...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        
        self.get_logger().info('Waiting for /delete_entity service...')
        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Delete service not available, waiting...')
        
        self.get_logger().info('All services are ready!')
    
    def generate_random_position(self):
        """Generate random x,y position within boundaries"""
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        return x, y
    
    def check_collision(self, x, y, min_distance=0.1):
        """Check if new position collides with existing boxes"""
        for box_info in self.spawned_boxes:
            existing_x = box_info['pose']['x']
            existing_y = box_info['pose']['y']
            
            distance = ((x - existing_x)**2 + (y - existing_y)**2)**0.5
            if distance < min_distance:
                return True
        return False
    
    def spawn_box(self, color, box_name, x, y):
        """Spawn a single colored box"""
        try:
            # Create spawn request
            request = SpawnEntity.Request()
            request.name = box_name
            
            # Get URDF file path
            package_name = "conveyorbelt_gazebo"  # Change this to your package name
            urdf_filename = f"box_{color}.urdf"
            urdf_file_path = os.path.join(
                get_package_share_directory(package_name), 
                'urdf', 
                urdf_filename
            )
            
            # Process URDF
            xacro_file = xacro.process_file(urdf_file_path)
            request.xml = xacro_file.toxml()
            
            # Set position
            request.initial_pose.position.x = float(x)
            request.initial_pose.position.y = float(y)
            request.initial_pose.position.z = float(self.z_fixed)
            
            # Set orientation (no rotation)
            request.initial_pose.orientation.x = 0.0
            request.initial_pose.orientation.y = 0.0
            request.initial_pose.orientation.z = 0.0
            request.initial_pose.orientation.w = 1.0
            
            # Call spawn service
            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'Successfully spawned {box_name} at ({x:.6f}, {y:.6f}, {self.z_fixed:.6f})')
                    
                    # Store box information
                    box_info = {
                        'name': box_name,
                        'color': color,
                        'pose': {'x': x, 'y': y, 'z': self.z_fixed}
                    }
                    self.spawned_boxes.append(box_info)
                    
                    # Publish box info with timestamp
                    self.publish_box_info(box_info)
                    
                    return True
                else:
                    self.get_logger().error(f'Failed to spawn {box_name}: {future.result().status_message}')
                    return False
            else:
                self.get_logger().error(f'Service call failed for {box_name}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error spawning {box_name}: {str(e)}')
            return False
    
    def delete_box(self, box_name):
        """Delete a single box"""
        try:
            request = DeleteEntity.Request()
            request.name = box_name
            
            future = self.delete_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'Successfully deleted {box_name}')
                    return True
                else:
                    self.get_logger().error(f'Failed to delete {box_name}: {future.result().status_message}')
                    return False
            else:
                self.get_logger().error(f'Delete service call failed for {box_name}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error deleting {box_name}: {str(e)}')
            return False
    
    def delete_all_boxes(self):
        """Delete all spawned boxes"""
        if not self.spawned_boxes:
            self.get_logger().info('No boxes to delete')
            return
        
        self.get_logger().info(f'Deleting all {len(self.spawned_boxes)} spawned boxes...')
        
        deleted_count = 0
        boxes_to_delete = self.spawned_boxes.copy()  # Create a copy to iterate over
        
        for box_info in boxes_to_delete:
            if self.delete_box(box_info['name']):
                deleted_count += 1
                self.spawned_boxes.remove(box_info)
        
        self.get_logger().info(f'Successfully deleted {deleted_count} boxes')
        
        # Publish empty info to indicate all boxes are deleted
        empty_msg = BoxInfo()
        empty_msg.header.stamp = self.get_clock().now().to_msg()
        empty_msg.header.frame_id = "world"
        empty_msg.name = "ALL_BOXES_DELETED"
        empty_msg.color = ""
        # Leave pose as default zeros
        self.box_info_publisher.publish(empty_msg)
    
    def publish_box_info(self, box_info):
        """Publish box information using custom BoxInfo message with timestamp"""
        box_msg = BoxInfo()
        
        # Set header with current timestamp - CRITICAL for synchronization
        box_msg.header.stamp = self.get_clock().now().to_msg()
        box_msg.header.frame_id = "world"
        
        # Set box info
        box_msg.name = box_info['name']
        box_msg.color = box_info['color']
        
        # Set pose
        box_msg.pose.position.x = float(box_info['pose']['x'])
        box_msg.pose.position.y = float(box_info['pose']['y'])
        box_msg.pose.position.z = float(box_info['pose']['z'])
        
        # Set orientation (no rotation)
        box_msg.pose.orientation.x = 0.0
        box_msg.pose.orientation.y = 0.0
        box_msg.pose.orientation.z = 0.0
        box_msg.pose.orientation.w = 1.0
        
        self.box_info_publisher.publish(box_msg)
        self.get_logger().debug(f'Published BoxInfo for {box_info["name"]} with timestamp')
    
    def publish_all_boxes_info(self):
        """Publish information for all spawned boxes"""
        for box_info in self.spawned_boxes:
            self.publish_box_info(box_info)
    
    def spawn_random_boxes(self, num_boxes=5, max_attempts=100):
        """Spawn multiple boxes randomly - one of each color, limited to 1-5 boxes"""
        # Limit number of boxes to 1-5
        num_boxes = max(1, min(5, num_boxes))
        
        self.get_logger().info(f'Starting to spawn {num_boxes} different colored boxes...')
        
        # Select the first num_boxes colors to ensure different colors
        selected_colors = self.box_colors[:num_boxes]
        successful_spawns = 0
        
        for i, color in enumerate(selected_colors):
            # Simple naming: box_color_number (e.g., box_red_1, box_blue_2)
            box_name = f"box_{color}_{i+1}"
            
            # Try to find a valid position
            attempts = 0
            while attempts < max_attempts:
                x, y = self.generate_random_position()
                
                # Check for collisions
                if not self.check_collision(x, y):
                    if self.spawn_box(color, box_name, x, y):
                        successful_spawns += 1
                        break
                
                attempts += 1
            
            if attempts >= max_attempts:
                self.get_logger().warn(f'Could not find valid position for {box_name} after {max_attempts} attempts')
            
            # Small delay between spawns
            time.sleep(0.5)
        
        self.get_logger().info(f'Spawning complete! Successfully spawned {successful_spawns}/{num_boxes} boxes')
        return successful_spawns
    
    def print_spawned_boxes(self):
        """Print information about all spawned boxes"""
        self.get_logger().info("=== SPAWNED BOXES SUMMARY ===")
        for box in self.spawned_boxes:
            self.get_logger().info(
                f"Name: {box['name']} | Color: {box['color']} | "
                f"Position: ({box['pose']['x']:.6f}, {box['pose']['y']:.6f}, {box['pose']['z']:.6f})"
            )

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Spawn random colored boxes in Gazebo')
    parser.add_argument('--num_boxes', type=int, default=5, help='Number of boxes to spawn (1-5, one of each color)')
    parser.add_argument('--x_min', type=float, default=0.6, help='Minimum X coordinate')
    parser.add_argument('--x_max', type=float, default=0.9, help='Maximum X coordinate')
    parser.add_argument('--y_min', type=float, default=-0.1, help='Minimum Y coordinate')
    parser.add_argument('--y_max', type=float, default=0.2, help='Maximum Y coordinate')
    parser.add_argument('--z', type=float, default=0.76, help='Fixed Z coordinate')
    parser.add_argument('--package', type=str, default='conveyorbelt_gazebo/msg', help='Package name containing URDF files')
    
    args, unknown = parser.parse_known_args()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create spawner node
    spawner = RandomBoxSpawner()
    
    # Update parameters from command line
    spawner.x_min = args.x_min
    spawner.x_max = args.x_max
    spawner.y_min = args.y_min
    spawner.y_max = args.y_max
    spawner.z_fixed = args.z
    
    try:
        # Wait for services
        spawner.wait_for_services()
        
        # Spawn boxes
        spawned_count = spawner.spawn_random_boxes(args.num_boxes)
        
        # Print summary
        spawner.print_spawned_boxes()
        
        # Keep node alive and publish box information periodically
        spawner.get_logger().info('Node is running. Commands:')
        spawner.get_logger().info('  - Press "d" + Enter to delete all boxes')
        spawner.get_logger().info('  - Press "q" + Enter to quit')
        spawner.get_logger().info('  - Press Ctrl+C to force stop')
        
        # Create timer to publish all boxes info periodically (every 0.1 seconds for better sync)
        timer = spawner.create_timer(0.1, spawner.publish_all_boxes_info)
        
        # Spin the node
        while spawner.running and rclpy.ok():
            rclpy.spin_once(spawner, timeout_sec=0.1)
        
    except KeyboardInterrupt:
        spawner.get_logger().info('Keyboard interrupt received. Shutting down...')
    finally:
        spawner.running = False
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()