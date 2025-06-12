# 🚀 ros2cv – Real-Time Computer Vision for ROS 2 Robots! 🎥🤖

**ros2cv** is an open-source, modular ROS 2 package written in Python that enables **real-time computer vision** capabilities using OpenCV, [CvZone](https://github.com/cvzone/cvzone), and [MediaPipe](https://mediapipe.dev/). It's designed for simplicity, performance, and seamless ROS 2 integration.

> 🧠 **No classes required** — just clean, functional Python nodes for each vision task!

---

## 🔍 Features

This package provides multiple ROS 2 nodes, each performing a specific CV task:

- ✅ **Hand and Finger Detection**
- ✅ **Pose Estimation**
- ✅ **Face Mesh Landmark Tracking**
- ✅ **Classic Face Detection** (Haar Cascade)
- ✅ **Object Detection** (YOLO-ready)
- ✅ **Camera Publisher & Viewer Nodes**

Each node is:
- 🧵 Lightweight and real-time except (YOLO as it requires GPU and other settings)
- 🔄 Subscribes to `/camera/image/compressed`
- 📤 Publishes:
  - Annotated images
  - Processed data (landmarks, positions, etc.)
- 🧩 Easy to modify or extend

---

## 📖 Getting Started

### ✅ Prerequisites
- ROS 2 (Humble or later recommended)
- Python 3.8+
- `opencv-python`, `cvzone`, `mediapipe` , 'ultralytics'

Install dependencies:
```bash
pip install opencv-python mediapipe cvzone ultralytics
```

### 🚀 Launch a node
Example:
```bash
ros2 run ros2cv face_detection
```

---

## 💡 Perfect For
- Robotics developers
- CV research prototypes
- ROS 2 learners
- Anyone building real-time vision systems

---

## 🙌 Contributing

Pull requests are welcome! Feel free to suggest new modules or features.

---

## 🔗 Links

- 📘 [MediaPipe Documentation](https://mediapipe.dev/)
- 🧰 [CvZone GitHub](https://github.com/cvzone/cvzone)
- 📸 [OpenCV](https://opencv.org/)

⭐️ **Star this repo** to follow updates and support the project!

---

## 📜 License
This project is licensed under the MIT License.
