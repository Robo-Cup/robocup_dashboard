#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__('first_node')
        self.create_timer(1, self.timer_callback)
        self.create_timer(2, self.timer_callback2)

    def timer_callback(self):
        self.get_logger().info("CHIMKEN!")
    
    def timer_callback2(self):
        self.get_logger().info("asdf CHIMKEN!")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()