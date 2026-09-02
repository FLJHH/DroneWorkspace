#!/usr/bin/env python3
"""Apply repeatable R1 visual noise and a fixed transport delay."""

from collections import deque
import copy
import math
import threading

import cv2
from cv_bridge import CvBridge
import numpy as np
import rospy
import tf2_geometry_msgs
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CameraInfo, Image, Imu


class R1ImageConditioner:
    def __init__(self):
        self.delay = float(rospy.get_param("~delay_s", 0.05))
        self.noise_std = float(rospy.get_param("~noise_std", 1.5))
        self.brightness_amplitude = float(
            rospy.get_param("~brightness_amplitude", 0.03))
        self.brightness_hz = float(rospy.get_param("~brightness_hz", 0.2))
        self.blur_pixels = int(rospy.get_param("~blur_pixels", 1))
        self.dropout_rate = float(rospy.get_param("~dropout_rate", 0.0))
        self.backlight_alpha = float(rospy.get_param("~backlight_alpha", 1.0))
        self.backlight_beta = float(rospy.get_param("~backlight_beta", 0.0))
        self.occlusion_fraction = float(rospy.get_param("~occlusion_fraction", 0.0))
        self.exposure_step_amplitude = float(
            rospy.get_param("~exposure_step_amplitude", 0.0))
        self.exposure_step_period = float(
            rospy.get_param("~exposure_step_period", 6.0))
        self.burst_dropout_period = float(
            rospy.get_param("~burst_dropout_period", 0.0))
        self.burst_dropout_duration = float(
            rospy.get_param("~burst_dropout_duration", 0.0))
        self.delay_jitter = float(rospy.get_param("~delay_jitter_s", 0.0))
        self.exposure_time = float(rospy.get_param("~exposure_time_s", 0.0))
        self.focal_px = float(rospy.get_param("~focal_px", 1108.5))
        self.angular_dropout_scale = float(
            rospy.get_param("~angular_dropout_scale", 0.0))
        self.dropout_max = float(rospy.get_param("~dropout_max", 1.0))
        self.tag_occlusion_fraction = float(
            rospy.get_param("~tag_occlusion_fraction", 0.0))
        self.tag_size = float(rospy.get_param("~tag_size_m", 0.6))
        self.angular_velocity = (0.0, 0.0, 0.0)
        self.local_pose = None
        self.vehicle_world = None
        self.iris_world = None
        self.vehicle_velocity = (0.0, 0.0)
        self.iris_velocity = (0.0, 0.0)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.random = np.random.RandomState(int(rospy.get_param("~seed", 20260902)))
        self.bridge = CvBridge()
        cv2.setRNGSeed(int(rospy.get_param("~seed", 20260902)))
        self.lock = threading.Lock()
        self.images = deque()
        self.latest_camera_info = None
        self.start = rospy.Time.now()

        image_in = rospy.get_param("~image_in", "/iris_0/camera/image_raw")
        info_in = rospy.get_param("~camera_info_in", "/iris_0/camera/camera_info")
        image_out = rospy.get_param("~image_out", "/iris_0/camera/r1/image_raw")
        info_out = rospy.get_param("~camera_info_out", "/iris_0/camera/r1/camera_info")
        self.image_pub = rospy.Publisher(image_out, Image, queue_size=2)
        self.info_pub = rospy.Publisher(info_out, CameraInfo, queue_size=2)
        rospy.Subscriber(image_in, Image, self.image_cb, queue_size=3,
                         buff_size=16 * 1024 * 1024)
        rospy.Subscriber(info_in, CameraInfo, self.info_cb, queue_size=10)
        rospy.Subscriber("/iris_0/mavros/imu/data", Imu, self.imu_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.models_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.005), self.publish_due)
        rospy.loginfo("Image conditioner: delay=%.3fs noise_std=%.2f "
                      "brightness=+/-%.1f%% blur=%d dropout=%.1f%% occlusion=%.1f%%",
                      self.delay, self.noise_std, 100.0 * self.brightness_amplitude,
                      self.blur_pixels, 100.0 * self.dropout_rate,
                      100.0 * self.occlusion_fraction)

    def image_cb(self, msg):
        try:
            elapsed = (rospy.Time.now() - self.start).to_sec()
            angular_rate = math.sqrt(sum(v * v for v in self.angular_velocity))
            if self.vehicle_world is not None and self.iris_world is not None:
                distance = max(1.0, math.sqrt(
                    (self.vehicle_world.x - self.iris_world.x) ** 2 +
                    (self.vehicle_world.y - self.iris_world.y) ** 2 +
                    (self.vehicle_world.z - self.iris_world.z) ** 2))
                relative_speed = math.hypot(
                    self.vehicle_velocity[0] - self.iris_velocity[0],
                    self.vehicle_velocity[1] - self.iris_velocity[1])
                angular_rate += relative_speed / distance
            dropout = min(self.dropout_max,
                          self.dropout_rate + self.angular_dropout_scale * angular_rate)
            in_burst = (self.burst_dropout_period > 0.0 and
                        elapsed % self.burst_dropout_period <
                        self.burst_dropout_duration)
            if in_burst or self.random.rand() < dropout:
                return
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gain = 1.0 + self.brightness_amplitude * math.sin(
                2.0 * math.pi * self.brightness_hz * elapsed)
            if self.exposure_step_amplitude > 0.0:
                half_period = max(0.1, self.exposure_step_period / 2.0)
                direction = 1.0 if int(elapsed / half_period) % 2 == 0 else -1.0
                gain *= 1.0 + direction * self.exposure_step_amplitude
            conditioned = cv2.convertScaleAbs(
                image, alpha=gain * self.backlight_alpha,
                beta=self.backlight_beta)
            blur_pixels = self.blur_pixels
            if self.exposure_time > 0.0:
                blur_pixels = max(1, int(round(
                    self.focal_px * angular_rate * self.exposure_time)))
                blur_pixels = min(11, blur_pixels)
            if blur_pixels > 1:
                size = blur_pixels if blur_pixels % 2 else blur_pixels + 1
                kernel = np.zeros((size, size), dtype=np.float32)
                wx, wy, _wz = self.angular_velocity
                angle = math.atan2(wy, wx) if abs(wx) + abs(wy) > 1e-6 else 0.0
                radius = size // 2
                dx, dy = int(round(radius * math.cos(angle))), int(round(radius * math.sin(angle)))
                cv2.line(kernel, (radius - dx, radius - dy),
                         (radius + dx, radius + dy), 1.0, 1)
                kernel /= max(1.0, kernel.sum())
                conditioned = cv2.filter2D(conditioned, -1, kernel)
            noise = np.empty(conditioned.shape, dtype=np.int16)
            cv2.randn(noise, 0, self.noise_std)
            conditioned = cv2.add(conditioned, noise, dtype=cv2.CV_8U)
            if self.occlusion_fraction > 0.0:
                height, width = conditioned.shape[:2]
                strip = max(1, int(width * self.occlusion_fraction))
                center = int((0.5 + 0.35 * math.sin(0.11 * elapsed)) * width)
                left = max(0, min(width - strip, center - strip // 2))
                conditioned[:, left:left + strip] = 35
            if self.tag_occlusion_fraction > 0.0:
                self.occlude_projected_tag(conditioned, msg.header)
            output = self.bridge.cv2_to_imgmsg(conditioned, encoding="bgr8")
            output.header = msg.header
            with self.lock:
                info = copy.deepcopy(self.latest_camera_info)
                if info is not None:
                    info.header = msg.header
                    jitter = self.random.uniform(-self.delay_jitter,
                                                 self.delay_jitter)
                    actual_delay = max(0.0, self.delay + jitter)
                    self.images.append((rospy.Time.now() + rospy.Duration(actual_delay),
                                        output, info))
        except (ValueError, cv2.error) as error:
            rospy.logwarn_throttle(2.0, "R1 image conversion failed: %s", error)

    def info_cb(self, msg):
        with self.lock:
            self.latest_camera_info = msg

    def imu_cb(self, msg):
        a = msg.angular_velocity
        self.angular_velocity = (a.x, a.y, a.z)

    def pose_cb(self, msg):
        self.local_pose = msg.pose.position

    def models_cb(self, msg):
        try:
            vehicle_index = msg.name.index("ugv_0")
            iris_index = msg.name.index("iris_0")
            self.vehicle_world = msg.pose[vehicle_index].position
            self.iris_world = msg.pose[iris_index].position
            vehicle_twist = msg.twist[vehicle_index].linear
            iris_twist = msg.twist[iris_index].linear
            self.vehicle_velocity = (vehicle_twist.x, vehicle_twist.y)
            self.iris_velocity = (iris_twist.x, iris_twist.y)
        except ValueError:
            pass

    def occlude_projected_tag(self, image, header):
        if (self.local_pose is None or self.vehicle_world is None or
                self.iris_world is None or self.latest_camera_info is None):
            return
        origin_x = self.iris_world.x - self.local_pose.x
        origin_y = self.iris_world.y - self.local_pose.y
        point = PointStamped()
        point.header.stamp = header.stamp
        point.header.frame_id = "map"
        point.point.x = self.vehicle_world.x - origin_x
        point.point.y = self.vehicle_world.y - origin_y
        point.point.z = 2.23
        try:
            camera_point = self.tf_buffer.transform(
                point, "iris_0/camera_link", rospy.Duration(0.01))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        z = camera_point.point.z
        if z <= 0.1:
            return
        info = self.latest_camera_info
        fx, fy, cx, cy = info.K[0], info.K[4], info.K[2], info.K[5]
        u = int(fx * camera_point.point.x / z + cx)
        v = int(fy * camera_point.point.y / z + cy)
        tag_px = max(2, int(self.focal_px * self.tag_size / z))
        cover_h = max(1, int(tag_px * self.tag_occlusion_fraction))
        left, right = max(0, u - tag_px // 2), min(image.shape[1], u + tag_px // 2)
        top, bottom = max(0, v - cover_h // 2), min(image.shape[0], v + cover_h // 2)
        if left < right and top < bottom:
            image[top:bottom, left:right] = 45

    def publish_due(self, _event):
        now = rospy.Time.now()
        images = []
        with self.lock:
            while self.images and self.images[0][0] <= now:
                images.append(self.images.popleft())
        for _due, image_msg, info_msg in images:
            self.image_pub.publish(image_msg)
            self.info_pub.publish(info_msg)


if __name__ == "__main__":
    rospy.init_node("r1_image_conditioner")
    R1ImageConditioner()
    rospy.spin()
