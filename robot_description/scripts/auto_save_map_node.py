#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap

class AutoSaveMapNode(Node):
    def __init__(self):
        super().__init__('auto_save_map_node')
        self.cli = self.create_client(SaveMap, '/map_saver_server/save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /map_saver_server/save_map service...')

        self.timer = self.create_timer(10.0, self.save_map)

    def save_map(self):
        req = SaveMap.Request()
        req.map_topic = 'map'
        req.map_url = '/home/sunrise/maps/auto_saved_map'
        self.get_logger().info('Calling /map_saver_server/save_map...')
        self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = AutoSaveMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
