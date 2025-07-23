# YOLO Object Following with ROS 2

A ROS 2-based project that combines real-time object detection with camera feeds, dynamic target selection, and system resource monitoring.  
Built using [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) and `rclpy`, this repo demonstrates object tracking, user interaction, and motion direction estimation based on image data.

---

## Project Structure
ros2_workspace/
|- src/
|--- [basic_task](ros2_workspace/src/basic_task)    # A basic ROS2 Publisher and subscriber to CPU load data
|------ [basic_task](ros2_workspace/src/basic_task/basic_task)
|--------- __init__.py
|--------- hardware_data_pub.py 
|--------- hardware_data_sub.py
|------ resource
|------ [README.md](ros2_workspace/src/basic_task/README.md)
|------ package.xml
|------ running_example.mp4
|------ setup.cfg
|------ setup.py
|--- [advanced_task](ros2_workspace/src/advanced_task) # YOLO-based object detection and movement logic
|------ [advanced_task](ros2_workspace/src/advanced_task/advanced_task)
|--------- LLM_integration.py
|--------- RGB_publisher.py 
|--------- hardware_data_sub.py
|--------- detect_cameras.py
|------ resource
|------ Multiple_Detections.mp4
|------ Multiple_Detections_2.mp4
|------ [README.md](ros2_workspace/src/advanced_task/README.md)
|------ package.xml
|------ running_example.mp4
|------ setup.cfg
|------ setup.py
|------ target_change.mp4
## 📦 Features

### ✅ Advanced Task
- Real-time camera feed from OpenCV.
- YOLOv8 inference using GPU (if available).
- Detects target class (e.g., "person", "bottle") dynamically.
- Visual feedback using ROS2 `sensor_msgs/Image`.
- Publishes:
  - Annotated images
  - Detected target metadata (JSON)
  - Suggested movement direction (e.g., "move left", "stay still")

### ✅ Basic Task
- Monitors system CPU usage using `psutil`.
- Publishes CPU load over ROS2.
- Subscriber node receives and logs the data.
- Useful for embedded diagnostics or feedback loops.

---

## 🎥 Demo Videos

- `Multiple_Detections.mp4` — Shows multi-object detection with labels  
- `target_change.mp4` — Demonstrates dynamic switching between bottle and person  
- `running_example.mp4` — Full end-to-end execution preview  

---

## 🚀 Quick Start

### 1. Install dependencies
```bash
pip install ultralytics opencv-python psutil
sudo apt install ros-<your-distro>-cv-bridge ros-<your-distro>-vision-msgs
```
---

##  Build your ROS2 workspace
```bash
cd ros2_workspace
colcon build
source install/setup.bash
```




