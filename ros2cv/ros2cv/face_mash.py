import rclpy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from cvzone.FaceMeshModule import FaceMeshDetector

bridge = CvBridge()
detector = FaceMeshDetector(maxFaces=1)

node = None
face_mesh_pub = None
landmarks_pub = None
landmark_count_pub = None

def callback(msg):
    frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    img, faces = detector.findFaceMesh(frame, draw=True)

    coords = []
    landmark_count = 0

    if faces:
        landmarks = faces[0]
        landmark_count = len(landmarks)
        for lm in landmarks:
            if isinstance(lm, (list, tuple)) and len(lm) == 2:
                coords.extend(lm)
            else:
                node.get_logger().warn(f"Malformed landmark entry: {lm}")

    cv2.putText(img, f'Landmarks: {landmark_count}', (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    node.get_logger().info(f'Face landmarks detected: {landmark_count}')

    img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')
    face_mesh_pub.publish(img_msg)

    landmarks_msg = Float32MultiArray()
    # Make sure coords is float32 list to avoid assertion errors
    landmarks_msg.data = np.array(coords, dtype=np.float32).tolist()
    landmarks_pub.publish(landmarks_msg)

    count_msg = Int32()
    count_msg.data = landmark_count
    landmark_count_pub.publish(count_msg)

def main():
    global node, face_mesh_pub, landmarks_pub, landmark_count_pub
    rclpy.init()
    node = rclpy.create_node('face_mesh_node')

    face_mesh_pub = node.create_publisher(Image, '/camera/cv_zone/face_mesh', 10)
    landmarks_pub = node.create_publisher(Float32MultiArray, '/face_mesh_landmarks', 10)
    landmark_count_pub = node.create_publisher(Int32, '/face_mesh_landmark_count', 10)

    node.create_subscription(CompressedImage, '/camera/image/compressed', callback, 10)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

