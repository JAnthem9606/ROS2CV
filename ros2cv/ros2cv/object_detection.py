import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

def main():
    rclpy.init()
    node = rclpy.create_node('yolo_compressed_pub')

    model = YOLO('yolov8n.pt')
    bridge = CvBridge()

    pub_img = node.create_publisher(Image, '/camera/yolo/object_detection', 10)
    pub_classes = node.create_publisher(String, '/detected_yolo_classes', 10)

    def callback(msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = model(frame)

        annotated = results[0].plot()

        # Extract class names
        classes = [model.names[int(cls)] for cls in results[0].boxes.cls.cpu().numpy()]

        # Publish class names as comma-separated string
        classes_msg = String()
        classes_msg.data = ','.join(classes)
        pub_classes.publish(classes_msg)

        # Publish annotated image
        annotated_msg = bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header.stamp = node.get_clock().now().to_msg()
        pub_img.publish(annotated_msg)
        #cv2.imshow("Object Detection",annotated)
        cv2.waitKey(1)

    node.create_subscription(CompressedImage, '/camera/image/compressed', callback, 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
