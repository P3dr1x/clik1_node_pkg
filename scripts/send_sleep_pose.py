#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

SLEEP_POSITIONS = [0.0, -1.8, 1.55, 0.0, 0.8, 0.0]

class SleepPosePublisher(Node):
    def __init__(self):
        super().__init__('send_sleep_pose')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.timer = self.create_timer(1.0, self.publish_sleep_pose)
        self.sent = False

    def publish_sleep_pose(self):
        if not self.sent:
            msg = Float64MultiArray()
            msg.data = SLEEP_POSITIONS
            self.publisher_.publish(msg)
            self.get_logger().info('Sleep pose command sent')
            self.sent = True
            # Shutdown after sending
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SleepPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()