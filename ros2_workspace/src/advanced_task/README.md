# advanced_task

## Overview

This is a simple ROS 2 package that demonstrates a Talker/Listener setup.

The **publisher node** reads your laptop's CPU load (using `psutil`) at a sampling rate (R1) and publishes it to the `/myHWtopic` at a publish rate (R2).  
The **subscriber node** listens on `/myHWtopic` and prints every message it receives to the console.

---

## Package Contents

- **rgb_publisher**:   Publisher node for RGB image
- **llm_integration**: Publisher node for RGB image with LLM integration

---

## Parameters

### rgb_publisher - Publisher node
| Name               | Default            | Description                             |
| ------------------ | ------------------ | --------------------------------------- |
| `publisher_topic`  | /camera/image_raw  | Topic to publish RGB image              |
| `camera_index`     | 0                  | The index of the camera                 |
| `publish_rate`     | 10                 | Rate in Hz to publish data on ROS topic |

### llm_integration - Publisher node
| Name                       | Default                | Description                                     |
| -------------------------- | ---------------------- | ----------------------------------------------- |
| `publisher_raw_topic`      | /camera/image_raw      | Topic to publish RGB image                      |
| `publisher_detected_topic` | /camera/image_detected | Topic to publish RGB image with LLM integration |
| `camera_index`             | 0                      | The index of the camera.                        |
| `publish_rate`             | 10                     | Rate in Hz to publish data on ROS topic         |
| `target_class`             | person                 | The Class to detect                             |

---

## How to Build

Open a terminal, navigate to your workspace root and run:

```bash
colcon build
source install/setup.bash
```

---

## How to Run

**Not sure which camera index to use? Follow this steps**
1. Open a terminal, navigate to your workspace root and run:

```bash
cd ros2_workspace/src/advanced_task/advanced_task/
python3 detect_cameras.py
```
This script will list all the connected camera indexes.

Example output:

```bash
Available cameras:
  Index 0: C922 Pro Stream Webcam
  Index 2: Microsoft® LifeCam HD-3000: Mi
```
2. If your desired camera is not at Index 0, update the value of camera_index in the script to the correct index.









Open **two separate terminals**:

###  Terminal 1 - Run the Publisher Node

This node samples your CPU load at `sampling_rate` (R1) and publishes to `/myHWtopic` at `publish_rate` (R2):

```bash
ros2 run basic_task hardware_data_pub
```

You can also override parameters using:

```bash
ros2 run basic_task hardware_data_pub --ros-args -p sampling_rate:=2.0 -p publish_rate:=1.0
```

Example output:

```bash
[INFO] [hardware_data_pub_node]: Published: 'CPU Load: 18.7%'
```

###  Terminal 2 - Run the Subscriber Node
**NOTE: In the second terminal you should also run**

```bash
source install/setup.bash
```

This node listens on /myHWtopic and prints received messages to stdout:

```bash
ros2 run basic_task hardware_data_sub
```

Example output:

```bash
[INFO] [hardware_data_sub_node]: Received: 'CPU Load: 18.7%'
```






ros2 topic pub /target_class std_msgs/String "data: 'bowl'"