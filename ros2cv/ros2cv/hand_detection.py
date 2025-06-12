import rclpy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector

bridge = CvBridge()
detector = HandDetector(detectionCon=0.8, maxHands=2)
pub = None  # Publisher will be initialized in main()

def callback(msg):
    frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    hands, img = detector.findHands(frame)
    for hand in hands:
        print(f"{hand['type']}, {len(hand['lmList'])}")

    # Show image (optional)
    cv2.imshow("Hand Detection", img)
    cv2.waitKey(1)

    # Publish the image with detections
    img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    pub.publish(img_msg)

def main():
    global pub
    rclpy.init()
    node = rclpy.create_node('hand_detection_node')

    # Publisher for annotated image
    pub = node.create_publisher(Image, 'camera/cv_zone/hand_detection', 10)

    # Subscriber for compressed camera feed
    node.create_subscription(CompressedImage, '/camera/image/compressed', callback, 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

