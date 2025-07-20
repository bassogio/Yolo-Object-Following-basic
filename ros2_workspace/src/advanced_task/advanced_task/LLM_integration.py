#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import json
import torch
from ultralytics import YOLO

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # ----------------------------
        # ROS2 parameters 
        # ----------------------------
        self.declare_parameter('publisher_raw_topic',              '/camera/image_raw')
        self.declare_parameter('publisher_detected_topic',         '/camera/detected_target')
        self.declare_parameter('publisher_moving_direction_topic', '/robotAction')
        self.declare_parameter('publisher_meta_topic',             '/detections')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('target_id',    '/selected_target_id')

        self.raw_topic        = self.get_parameter('publisher_raw_topic').value
        self.detected_topic   = self.get_parameter('publisher_detected_topic').value
        self.moving_dir_topic = self.get_parameter('publisher_moving_direction_topic').value
        self.meta_topic       = self.get_parameter('publisher_meta_topic').value
        self.camera_index     = self.get_parameter('camera_index').value
        self.publish_rate     = self.get_parameter('publish_rate').value
        self.target_class     = self.get_parameter('target_class').value.strip().lower()
        self.target_id        = self.get_parameter('target_id').value

        self.get_logger().info(f"Starting with target_class='{self.target_class}'")
        self.get_logger().info(f"Raw images: {self.raw_topic}")
        self.get_logger().info(f"Detected target: {self.detected_topic}")
        self.get_logger().info(f"Moving direction: {self.moving_dir_topic}")

        # ----------------------------
        # Publishers & subscriber
        # ----------------------------
        self.bridge = CvBridge()
        self.raw_pub        = self.create_publisher(Image,  self.raw_topic,        10)
        self.detected_pub   = self.create_publisher(Image,  self.detected_topic,   10)
        self.meta_pub       = self.create_publisher(String, self.meta_topic,       10)
        self.moving_dir_pub = self.create_publisher(String, self.moving_dir_topic, 10)

        # listen for dynamic target updates
        self.create_subscription(String, '/target_class', self.target_callback, 10)
        self.get_logger().info(f"Subscribed to {self.target_class} for updates")

        # listen for user selection of target id
        self.selected_target_id = None
        self.create_subscription(String, self.target_id, self.selection_callback, 10)
        self.get_logger().info(f"Subscribed to {self.target_id} for user selection")

        # ----------------------------
        # Prepare YOLO on GPU if possible
        # ----------------------------
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")
        self.model = YOLO('yolo11n.pt')
        self.model.to(self.device)

        # ----------------------------
        # OpenCV video capture
        # ----------------------------
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            raise RuntimeError("Camera not available.")
        else:
            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"Camera opened with: {self.frame_width}x{self.frame_height} resolution")

        # ----------------------------
        # Timer
        # ----------------------------
        period = 1.0 / float(self.publish_rate)
        self.create_timer(period, self.publish_image)

    def target_callback(self, msg: String):
        """Callback to update the target class dynamically."""
        new = msg.data.strip().lower()
        if new:
            self.target_class = new
            self.get_logger().info(f"Updated target_class: '{self.target_class}'")

    def selection_callback(self, msg: String):
        """Callback to update selected target id from user."""
        try:
            self.selected_target_id = int(msg.data)
            self.get_logger().info(f"User selected target id: {self.selected_target_id}")
        except ValueError:
            self.get_logger().warn("Invalid target id received.")

    def publish_image(self):
        """Capture image, run YOLO inference, and publish results."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame.")
            return

        # Convert BGR -> RGB for YOLO
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 

        # Publish raw frame
        raw_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8') # Convert to ROS Image message 
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        self.raw_pub.publish(raw_msg)

        # Run inference
        results = self.model(
            rgb,               # Input image in RGB format
            device=self.device # Use GPU if available
        )

        # Draw boxes & collect metadata
        annotated = frame.copy() # Copy original frame for annotations
        detections = []          # List to hold detection metadata
        target_index = 0         # Counter for target class instances

        for box in results[0].boxes: 
            cls_id     = int(box.cls[0])                    # Class ID
            name       = self.model.names[cls_id]           # Class name
            conf       = float(box.conf[0])                 # Confidence score
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist()) # Get bounding box coordinates
            midpoint = ((x1 + x2) // 2, (y1 + y2) // 2)     # Calculate midpoint

            if name.lower() == self.target_class:
                target_index += 1
                # draw
                #                        top-left bottom-right color thickness   
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,255,0), 2)
                #                     center  radius color    fill
                cv2.circle(annotated, midpoint, 5, (0,0,255), -1)
                label = f"{name} #{target_index} {conf:.2f}"
                cv2.putText(annotated, label, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
                detections.append({
                    "id": target_index,
                    "class": name,
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2],
                    "midpoint": midpoint
                })

        if len(detections) == 1:
            if self.selected_target_id is not None:
                self.selected_target_id = None # Reset selection if only one target detected
            direction = self.moving_direction(annotated, detections[0]['midpoint'])
        elif len(detections) > 1:
            # Wait for user selection
            selected = next((d for d in detections if d['id'] == self.selected_target_id), None)
            if selected:
                direction = self.moving_direction(annotated, selected['midpoint'])
            else:
                direction = "Wait for user selection"
        else:
            direction = "No target detected"

        # Publish annotated frame
        ann_rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)              # Convert back to RGB for ROS
        det_img_msg = self.bridge.cv2_to_imgmsg(ann_rgb, encoding='rgb8') # Convert to ROS Image message
        det_img_msg.header.stamp = self.get_clock().now().to_msg() 
        self.detected_pub.publish(det_img_msg)

        # Publish JSON metadata
        meta = String()
        meta.data = json.dumps(detections)
        self.meta_pub.publish(meta)
        if detections:
            self.get_logger().info(f"Detected {len(detections)} '{self.target_class}'")

        # Publish moving direction
        if detections:
            dir_msg = String()
            dir_msg.data = direction
            self.moving_dir_pub.publish(dir_msg)
            self.get_logger().info(f"Moving direction: {direction}")

    def moving_direction(self, annotated, midpoint, tolerance=20):
        """Determine moving direction based on midpoint of detected target, with tolerance"""

        # Calculate the center of the frame
        center_x = self.frame_width / 2
        center_y = self.frame_height / 2

        # Draw center point
        cv2.rectangle(
            annotated,
            (int(center_x - tolerance), int(center_y - tolerance)),
            (int(center_x + tolerance), int(center_y + tolerance)),
            (255, 0, 0), 2
        )

        # Error checking for midpoint
        dx = midpoint[0] - center_x 
        dy = midpoint[1] - center_y

        if abs(dx) <= tolerance and abs(dy) <= tolerance:
            return "Stay still"
        elif abs(dx) <= tolerance:
            if dy < -tolerance:
                return "Move up"
            elif dy > tolerance:
                return "Move down"
        elif abs(dy) <= tolerance:
            if dx < -tolerance:
                return "Move left"
            elif dx > tolerance:
                return "Move right"
        else:
            if dx < -tolerance and dy < -tolerance:
                return "Move up and left"
            elif dx < -tolerance and dy > tolerance:
                return "Move down and left"
            elif dx > tolerance and dy < -tolerance:
                return "Move up and right"
            elif dx > tolerance and dy > tolerance:
                return "Move down and right"
        return "Unknown direction"

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
