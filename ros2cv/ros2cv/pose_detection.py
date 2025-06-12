import rclpy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from cvzone.PoseModule import PoseDetector

bridge = CvBridge()
detector = PoseDetector()

node = None
pose_pub = None
landmarks_pub = None
landmark_count_pub = None

def callback(msg):
    frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    img = detector.findPose(frame, draw=True)
    lmList = detector.findPosition(img, draw=False)

    coords = []
    landmark_count = 0

    if lmList:
        landmark_count = len(lmList)
        for lm in lmList:
            if isinstance(lm, (list, tuple)) and len(lm) >= 3:
                # Handle possible nested coords, e.g. lm[1] might be [x,y,z]
                if isinstance(lm[1], (list, tuple)):
                    coords.extend([float(lm[1][0]), float(lm[1][1])])
                else:
                    coords.extend([float(lm[1]), float(lm[2])])
            else:
                node.get_logger().warn(f"Malformed landmark entry: {lm}")

    cv2.putText(img, f'Landmarks: {landmark_count}', (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    node.get_logger().info(f'Pose landmarks detected: {landmark_count}')

    img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    pose_pub.publish(img_msg)

    landmarks_msg = Float32MultiArray()
    landmarks_msg.data = np.array(coords, dtype=np.float32).tolist()
    landmarks_pub.publish(landmarks_msg)

    count_msg = Int32()
    count_msg.data = landmark_count
    landmark_count_pub.publish(count_msg)

def main():
    global node, pose_pub, landmarks_pub, landmark_count_pub
    rclpy.init()
    node = rclpy.create_node('pose_estimation_node')

    pose_pub = node.create_publisher(Image, '/camera/cv_zone/pose_detection', 10)
    landmarks_pub = node.create_publisher(Float32MultiArray, '/pose_landmarks', 10)
    landmark_count_pub = node.create_publisher(Int32, '/pose_landmark_count', 10)

    node.create_subscription(CompressedImage, '/camera/image/compressed', callback, 10)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

