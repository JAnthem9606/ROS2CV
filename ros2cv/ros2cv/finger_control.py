import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

def main(args=None):
    rclpy.init(args=args)
    node = Node('finger_control_node')

    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def callback(msg):
        twist = Twist()
        if msg.data == 1:
            twist.linear.x = 0.2  # forward
        elif msg.data == 2:
            twist.linear.x = -0.2  # backward
        elif msg.data == 3:
            twist.angular.z = 0.5  # left
        elif msg.data == 4:
            twist.angular.z = -0.5  # right
        else:
            twist.linear.x = 0.0  # stop
            twist.angular.z = 0.0

        publisher.publish(twist)
        node.get_logger().info(f'Fingers: {msg.data} -> cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

    subscription = node.create_subscription(Int32, '/fingers_count', callback, 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
