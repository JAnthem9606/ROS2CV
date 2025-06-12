import rclpy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from cvzone.HandTrackingModule import HandDetector

bridge = CvBridge()
detector = HandDetector(detectionCon=0.8, maxHands=2)

node = None
video_pub = None
fingers_pub = None
count_pub = None

def callback(msg):
    frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    hands, img = detector.findHands(frame)
    
    fingers_nested = [detector.fingersUp(hand) for hand in hands]
    total_fingers_open = sum(sum(f) for f in fingers_nested)

    # Draw the number of open fingers on the frame
    cv2.putText(
        img,
        f'Fingers Open: {total_fingers_open}',
        (20, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    # Log per-hand fingers and total
    for i, fingers in enumerate(fingers_nested):
        node.get_logger().info(f'Hand {i+1}: {fingers}')
    node.get_logger().info(f'Total Fingers Open: {total_fingers_open}')

    # Publish flattened finger states
    fingers_msg = Float32MultiArray()
    fingers_msg.data = np.array(fingers_nested).flatten().astype(float).tolist()
    fingers_pub.publish(fingers_msg)

    # Publish total count
    count_msg = Int32()
    count_msg.data = total_fingers_open
    count_pub.publish(count_msg)

    # Publish annotated image
    img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    video_pub.publish(img_msg)

def main():
    global node, video_pub, fingers_pub, count_pub
    rclpy.init()
    node = rclpy.create_node('hand_tracking_node')

    video_pub = node.create_publisher(Image, 'camera/cv_zone/fingers_detection', 10)
    fingers_pub = node.create_publisher(Float32MultiArray, '/fingers_state', 10)
    count_pub = node.create_publisher(Int32, '/fingers_count', 10)

    node.create_subscription(CompressedImage, '/camera/image/compressed', callback, 10)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

