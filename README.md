# YOLO Object Following with ROS 2

A ROS 2-based project that combines real-time object detection with camera feeds, dynamic target selection, and system resource monitoring.  
Built using [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) and `rclpy`, this repo demonstrates object tracking, user interaction, and motion direction estimation based on image data.

---

## Project Structure

ros2_workspace/
├── src/
│   ├── [basic_task](src/basic_task)                     # CPU monitoring publisher & subscriber
│   │   ├── [basic_task](src/basic_task/basic_task)/
│   │   │   ├── [__init__.py](src/basic_task/basic_task/__init__.py)
│   │   │   ├── [hardware_data_pub.py](src/basic_task/basic_task/hardware_data_pub.py)
│   │   │   └── [hardware_data_sub.py](src/basic_task/basic_task/hardware_data_sub.py)
│   │   ├── [resource/](src/basic_task/resource/)
│   │   ├── [README.md](src/basic_task/README.md)
│   │   ├── [package.xml](src/basic_task/package.xml)
│   │   ├── [running_example.mp4](src/basic_task/running_example.mp4)
│   │   ├── [setup.cfg](src/basic_task/setup.cfg)
│   │   └── [setup.py](src/basic_task/setup.py)
│   └── [advanced_task](src/advanced_task)               # YOLO-based object detection & movement logic
│       ├── [advanced_task](src/advanced_task/advanced_task)/
│       │   ├── [LLM_integration.py](src/advanced_task/advanced_task/LLM_integration.py)
│       │   ├── [RGB_publisher.py](src/advanced_task/advanced_task/RGB_publisher.py)
│       │   ├── [hardware_data_sub.py](src/advanced_task/advanced_task/hardware_data_sub.py)
│       │   └── [detect_cameras.py](src/advanced_task/advanced_task/detect_cameras.py)
│       ├── [resource/](src/advanced_task/resource/)
│       ├── [Multiple_Detections.mp4](src/advanced_task/Multiple_Detections.mp4)
│       ├── [Multiple_Detections_2.mp4](src/advanced_task/Multiple_Detections_2.mp4)
│       ├── [README.md](src/advanced_task/README.md)
│       ├── [package.xml](src/advanced_task/package.xml)
│       ├── [running_example.mp4](src/advanced_task/running_example.mp4)
│       ├── [setup.cfg](src/advanced_task/setup.cfg)
│       ├── [setup.py](src/advanced_task/setup.py)
│       └── [target_change.mp4](src/advanced_task/target_change.mp4)

---

##  Features

###  Advanced Task
- Real-time camera feed from OpenCV.
- YOLOv8 inference using GPU (if available).
- Detects target class (e.g., "person", "bottle") dynamically.
- Visual feedback using ROS2 `sensor_msgs/Image`.
- Publishes:
  - Annotated images
  - Detected target metadata (JSON)
  - Suggested movement direction (e.g., "move left", "stay still")

###  Basic Task
- Monitors system CPU usage using `psutil`.
- Publishes CPU load over ROS2.
- Subscriber node receives and logs the data.
- Useful for embedded diagnostics or feedback loops.

---

##  Demo Videos

- `Multiple_Detections.mp4` — Shows multi-object detection with labels  
- `target_change.mp4` — Demonstrates dynamic switching between bottle and person  
- `running_example.mp4` — Full end-to-end execution preview  
