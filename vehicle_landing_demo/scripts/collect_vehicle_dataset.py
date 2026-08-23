#!/usr/bin/env python3
"""Collect and pseudo-label Gazebo vehicle images for one-class YOLO tuning."""

import os
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO


class Collector:
    CANDIDATE_NAMES = {
        "car", "truck", "bus", "boat", "cell phone", "oven", "microwave",
        "remote", "tv", "suitcase",
    }

    def __init__(self):
        self.output = rospy.get_param(
            "~output", "/home/fenglijun/DroneWorkspace/vehicle_landing_demo/datasets/vehicle")
        self.target_count = int(rospy.get_param("~count", 240))
        self.interval = float(rospy.get_param("~interval", 0.35))
        self.last_stamp = -1.0
        self.count = 0
        self.model = YOLO(rospy.get_param(
            "~weight", "/home/fenglijun/catkin_ws/src/yolov11_ros/weights/yolo11s.pt"))
        for split in ("train", "val"):
            os.makedirs(os.path.join(self.output, "images", split), exist_ok=True)
            os.makedirs(os.path.join(self.output, "labels", split), exist_ok=True)
        self.sub = rospy.Subscriber(
            "/iris_0/camera/image_raw", Image, self.callback,
            queue_size=1, buff_size=52428800)

    def callback(self, message):
        stamp = message.header.stamp.to_sec()
        if self.count >= self.target_count or stamp - self.last_stamp < self.interval:
            return
        frame = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, -1)
        if message.encoding.lower() in ("rgb8", "rgba8"):
            code = cv2.COLOR_RGB2BGR if frame.shape[2] == 3 else cv2.COLOR_RGBA2BGR
            frame = cv2.cvtColor(frame, code)
        elif frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        result = self.model(frame, imgsz=1280, conf=0.01, verbose=False)[0]
        candidates = []
        for box in result.boxes:
            name = result.names[int(box.cls.item())]
            if name not in self.CANDIDATE_NAMES:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            width, height = x2 - x1, y2 - y1
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            # The controlled rear-quarter observation keeps the vehicle in
            # the central 70% of the image and at a useful apparent size.
            if not (0.15 * message.width < cx < 0.85 * message.width and
                    0.15 * message.height < cy < 0.85 * message.height and
                    width > 45 and height > 55):
                continue
            confidence = float(box.conf.item())
            area_bonus = min(width * height / 25000.0, 1.0) * 0.10
            candidates.append((confidence + area_bonus, x1, y1, x2, y2, name, confidence))
        if not candidates:
            rospy.logwarn_throttle(2.0, "collector: no usable vehicle pseudo-box")
            return

        _, x1, y1, x2, y2, source_name, confidence = max(candidates)
        # Expand slightly because the wrong COCO class often clips bumpers.
        pad_x, pad_y = 0.04 * (x2 - x1), 0.04 * (y2 - y1)
        x1, y1 = max(0.0, x1 - pad_x), max(0.0, y1 - pad_y)
        x2, y2 = min(message.width - 1.0, x2 + pad_x), min(message.height - 1.0, y2 + pad_y)
        cx = (x1 + x2) / (2.0 * message.width)
        cy = (y1 + y2) / (2.0 * message.height)
        width = (x2 - x1) / message.width
        height = (y2 - y1) / message.height

        split = "val" if self.count % 5 == 0 else "train"
        stem = "vehicle_{:05d}".format(self.count)
        cv2.imwrite(os.path.join(self.output, "images", split, stem + ".jpg"), frame)
        with open(os.path.join(self.output, "labels", split, stem + ".txt"), "w") as label:
            label.write("0 {:.6f} {:.6f} {:.6f} {:.6f}\n".format(cx, cy, width, height))
        self.count += 1
        self.last_stamp = stamp
        rospy.loginfo(
            "collector: %d/%d %s <- %s %.2f", self.count, self.target_count,
            split, source_name, confidence)
        if self.count >= self.target_count:
            rospy.signal_shutdown("dataset complete")


if __name__ == "__main__":
    rospy.init_node("collect_vehicle_dataset")
    Collector()
    rospy.spin()
