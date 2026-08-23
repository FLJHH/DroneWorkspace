#!/usr/bin/env python3
"""YOLO detector tuned for high-altitude vehicle and person detection."""

import math
import os

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from ultralytics import YOLO
from yolov11_ros_msgs.msg import BoundingBox, BoundingBoxes


class VehicleYolo:
    # COCO: person=0, car=2, bus=5, truck=7.
    CLASSES = [0, 2, 5, 7]

    def __init__(self):
        self.frame_id = rospy.get_param("~camera_frame", "")
        self.conf = float(rospy.get_param("~conf", 0.30))
        self.imgsz = int(rospy.get_param("~imgsz", 1280))
        self.visualize = bool(rospy.get_param("~visualize", True))
        self.min_vehicle_aspect = float(rospy.get_param("~min_vehicle_aspect", 0.25))
        self.max_vehicle_aspect = float(rospy.get_param("~max_vehicle_aspect", 0.70))
        self.tag_handoff_height = float(rospy.get_param("~tag_handoff_height", 3.0))
        self.platform_height = float(rospy.get_param("~platform_height", 2.23))
        self.height_above_roof = float("inf")
        self.debug_dir = rospy.get_param("~debug_dir", "")
        self.last_debug_height = None
        if self.debug_dir:
            os.makedirs(self.debug_dir, exist_ok=True)
        device = "cpu" if rospy.get_param("/use_gpu", True) else "cuda"
        self.model = YOLO(rospy.get_param("~weight_path"))
        self.model.fuse()
        self.classes = [0] if len(self.model.names) == 1 else self.CLASSES
        self.box_pub = rospy.Publisher(
            rospy.get_param("~pub_topic", "/landing/yolo_boxes"),
            BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher("/yolov11/detection_image", Image, queue_size=1)
        self.sub = rospy.Subscriber(
            rospy.get_param("~image_topic", "/iris_0/camera/image_raw"),
            Image, self.image_cb, queue_size=1, buff_size=52428800)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_cb, queue_size=1)
        self.device = device

    def models_cb(self, message):
        """Altitude gate for the explicit YOLO-to-AprilTag handoff."""
        try:
            iris_z = message.pose[message.name.index("iris_0")].position.z
            self.height_above_roof = iris_z - self.platform_height
        except ValueError:
            pass

    def image_cb(self, image):
        # Ultralytics accepts OpenCV-style BGR arrays. Gazebo commonly publishes
        # rgb8, so honor the message encoding instead of assuming channel order.
        frame = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        if image.encoding.lower() in ("rgb8", "rgba8"):
            conversion = cv2.COLOR_RGB2BGR if frame.shape[2] == 3 else cv2.COLOR_RGBA2BGR
            frame = cv2.cvtColor(frame, conversion)
        elif frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        results = self.model(
            frame, show=False, conf=self.conf, imgsz=self.imgsz,
            classes=self.classes, device=self.device, verbose=False)
        result = results[0]
        raw_descriptions = []
        boxes = BoundingBoxes()
        boxes.header = image.header
        boxes.image_header = image.header
        accepted = []
        rejected = 0
        for detection in result.boxes:
            xyxy = detection.xyxy[0].tolist()
            width = max(1.0, xyxy[2] - xyxy[0])
            height = max(1.0, xyxy[3] - xyxy[1])
            aspect = width / height
            raw_descriptions.append("{:.2f}/{:.3f}".format(
                aspect, width * height / float(image.width * image.height)))
            # Reject implausibly thin/wide fragments while allowing the full
            # perspective range measured in real glideslope frames.
            if not self.min_vehicle_aspect <= aspect <= self.max_vehicle_aspect:
                rejected += 1
                continue
            # Below this height precision guidance belongs exclusively to the
            # AprilTag pipeline. Suppress YOLO vehicle output so the two target
            # semantics cannot be confused at touchdown.
            if self.height_above_roof <= self.tag_handoff_height:
                rejected += 1
                continue
            box = BoundingBox()
            box.xmin, box.ymin = int(xyxy[0]), int(xyxy[1])
            box.xmax, box.ymax = int(xyxy[2]), int(xyxy[3])
            box.Class = result.names[int(detection.cls.item())]
            box.probability = float(detection.conf.item())
            accepted.append((width * height, detection, box))
        # There is exactly one landing vehicle. When the network also proposes
        # roof/body fragments, publish the largest valid silhouette only.
        if accepted:
            accepted = [max(accepted, key=lambda item: item[0])]
            boxes.bounding_boxes.append(accepted[0][2])
        self.box_pub.publish(boxes)
        if accepted:
            best = float(accepted[0][1].conf.item())
            rospy.loginfo_throttle(
                1.0, "YOLO: full vehicle box confidence %.2f", best)
        else:
            rospy.logwarn_throttle(
                2.0, "YOLO: no valid vehicle boxes (raw=%d rejected=%d roof=%.2f shape=%s)",
                len(result.boxes), rejected, self.height_above_roof,
                ",".join(raw_descriptions) if raw_descriptions else "none")

        # Keep one real descent frame per integer altitude for dataset auditing.
        debug_height = (int(max(0.0, self.height_above_roof))
                        if math.isfinite(self.height_above_roof) else None)
        if (self.debug_dir and debug_height is not None and
                debug_height != self.last_debug_height and
                self.height_above_roof < 25.0):
            cv2.imwrite(os.path.join(
                self.debug_dir, "roof_{:02d}.jpg".format(debug_height)), frame)
            self.last_debug_height = debug_height

        # Draw only validated boxes. result.plot() would also visualize raw
        # rejected detections and make an AprilTag false positive appear valid.
        annotated = frame.copy()
        for _, detection, _ in accepted:
            x1, y1, x2, y2 = [int(value) for value in detection.xyxy[0].tolist()]
            confidence = float(detection.conf.item())
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 200, 0), 3)
            cv2.putText(annotated, "vehicle {:.2f}".format(confidence),
                        (x1, max(24, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 200, 0), 2, cv2.LINE_AA)
        if self.height_above_roof <= self.tag_handoff_height:
            cv2.putText(annotated, "AprilTag precision phase",
                        (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (0, 180, 255), 2, cv2.LINE_AA)
        output = Image()
        output.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        output.height, output.width = image.height, image.width
        output.encoding, output.step = "bgr8", image.width * 3
        output.data = annotated.tobytes()
        self.image_pub.publish(output)
        if self.visualize:
            cv2.imshow("YOLO vehicle/person", cv2.resize(
                annotated, (image.width // 2, image.height // 2)))
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("vehicle_yolo")
    VehicleYolo()
    rospy.spin()
