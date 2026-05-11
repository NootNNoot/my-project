#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class TorqueTest(Node):
    def __init__(self):
        super().__init__("torque_test")
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/arm_effort_controller/commands",
            10
        )
        self.timer = self.create_timer(0.01, self.loop)

    def loop(self):
        msg = Float64MultiArray()
        msg.data = [0.0, -80.0, 0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)
        self.get_logger().info(f"published {msg.data}")

def main():
    rclpy.init()
    node = TorqueTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()