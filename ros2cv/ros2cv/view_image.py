import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

def main():
    rclpy.init()
    node = rclpy.create_node('compressed_image_viewer')

    def callback(msg: CompressedImage):
        # Convert CompressedImage data to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Decode image from numpy array
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            cv2.imshow('Compressed Image', img)
            cv2.waitKey(1)

    node.create_subscription(CompressedImage, '/image/video/compressed', callback, 10)

    rclpy.spin(node)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

