#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

rclpy.init()
node = rclpy.create_node('face_tracker_follow_compressed')
bridge = CvBridge()
cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
image_pub = node.create_publisher(Image, '/face_tracker/image_annotated', 10)
twist = Twist()

def compressed_image_callback(msg: CompressedImage):
    global twist
    # Decode compressed image to OpenCV format
    np_arr = np.frombuffer(msg.data, np.uint8)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

    height, width = frame.shape[:2]
    center_x = width // 2

    annotated_frame = frame.copy()

    if len(faces) == 0:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_pub.publish(twist)
        node.get_logger().info('No face detected: Stopping')
        image_pub.publish(bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))
        return

    face = max(faces, key=lambda rect: rect[2] * rect[3])
    (x, y, w, h) = face
    face_center_x = x + w // 2

    cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    error_x = face_center_x - center_x

    linear_speed = 0.0
    angular_speed = 0.0

    if abs(error_x) > 30:
        angular_speed = -0.2 * error_x
        angular_speed = max(min(angular_speed, 0.5), -0.5)
        linear_speed = 0.0
        node.get_logger().info(f'Turning {"left" if angular_speed > 0 else "right"}')
    else:
        if w < 250:
            linear_speed = 0.2
            node.get_logger().info('Moving forward')
        elif w > 250:
            linear_speed = -0.1
            node.get_logger().info('Too close: moving backward')
        else:
            linear_speed = 0.0
            node.get_logger().info('Face centered and good distance: stopping')
        angular_speed = 0.0

    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    cmd_pub.publish(twist)

    image_pub.publish(bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))


node.create_subscription(CompressedImage,  '/camera/image/compressed', compressed_image_callback, 10)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
