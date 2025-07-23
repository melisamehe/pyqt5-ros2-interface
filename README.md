# 🔭 ROS 2 + PyQt5 GUI Interface for LiDAR & Multi-Camera System

This project is a real-time graphical user interface (GUI) developed using **PyQt5** that visualizes data from:
- 📷 **Three cameras** (forward, rear, aiming)
- 🧭 **LiDAR sensor** (2D LaserScan)

It is built upon **ROS 2 Humble** and supports real-time streaming of `sensor_msgs/Image` and `sensor_msgs/LaserScan` messages into a user-friendly window.

---<img width="1204" height="1006" alt="Screenshot from 2025-07-23 18-45-24" src="https://github.com/user-attachments/assets/5191a003-68bc-4e66-b9f7-9ce724be8528" />
<img width="1204" height="1006" alt="Screenshot from 2025-07-23 18-41-19" src="https://github.com/user-attachments/assets/345cbe6d-a7dd-49d0-9991-1ddeedd1c518" />



## 🚀 Features

- 🖥️ PyQt5-based responsive GUI
- 📡 ROS 2 Humble node integration inside a separate thread
- 🎥 Displays three live camera feeds:
  - `/camera/color/image_raw` (forward)
  - `/camera/rear/image_raw` (rear)
  - `/camera/aiming/image_raw` (aiming)
- 🌐 Real-time LiDAR scan visualizer with radial rendering
- ⚙️ Multi-threaded architecture using `QThread` to prevent GUI freeze
- 🎯 Scalable and modular design for future expansion

- ## 📷 GUI Layout Overview

+----------------------------+-----------------------------+
| | |
| LiDAR View | Forward Camera (RealSense)|
| | Rear Camera |
| | Aiming Camera |
+----------------------------+-----------------------------+


---

## 🛠️ Dependencies

- Python 3.8+
- ROS 2 Humble
- `cv_bridge`
- `rclpy`
- `PyQt5`
- `OpenCV`
- `numpy`

- You can install Python dependencies via:


pip install opencv-python PyQt5 numpy


- ▶️ Run the Interface

    Make sure your ROS 2 environment is sourced, and sensors are publishing.

ros2 run <your_package_name> camera_node

Or directly via Python:

python3 camera_node.py

    Replace <your_package_name> with the name used in your ROS 2 package.




🧠 How It Works

    ROS2Thread handles subscriptions to image and lidar topics in a separate thread using rclpy.spin().

    Each incoming ROS message is converted with cv_bridge and sent via pyqtSignal to the PyQt5 main thread.

    The main window (MergedInterface) updates camera views and draws a polar LiDAR view with OpenCV.
