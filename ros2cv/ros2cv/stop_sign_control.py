#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = rclpy.create_node('stop_sign_control')

    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def callback(msg):
        twist = Twist()
        data = msg.data.strip()
        objects = [obj.strip() for obj in data.split(',') if obj.strip() != '']

        if objects and objects[0] == 'stop sign':
            # stop
            twist.linear.x = 0.0  # stop
            twist.angular.z = 0.0
            node.get_logger().info('Stopping (stop sign detected)')
        else:
            # move forward
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            node.get_logger().info('Moving Forward (no stop sign infront)')

        pub.publish(twist)

    sub = node.create_subscription(String, '/detected_yolo_classes', callback, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
