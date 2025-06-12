import rclpy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()
node = None
image_pub = None
count_pub = None
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def callback(msg):
    frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1,
                                          minNeighbors=5,
                                          minSize=(30, 30))
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.putText(frame, f'Faces: {len(faces)}', (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    node.get_logger().info(f'Faces detected: {len(faces)}')

    img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    image_pub.publish(img_msg)

    count_msg = Int32()
    count_msg.data = len(faces)
    count_pub.publish(count_msg)

def main():
    global node, image_pub, count_pub
    rclpy.init()
    node = rclpy.create_node('face_detection_node')

    image_pub = node.create_publisher(Image, '/camera/face_detection/image', 10)
    count_pub = node.create_publisher(Int32, '/camera/face_detection/count', 10)

    node.create_subscription(CompressedImage, '/camera/image/compressed', callback, 10)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

