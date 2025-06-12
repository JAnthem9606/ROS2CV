import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def main():
    rclpy.init()
    node = rclpy.create_node('raw_webcam_publisher')
    pub = node.create_publisher(CompressedImage, '/camera/image/compressed', 10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    while rclpy.ok():
        _, frame = cap.read()
        frame = cv2.resize(frame,(640,480))
        msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.01)

    cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
