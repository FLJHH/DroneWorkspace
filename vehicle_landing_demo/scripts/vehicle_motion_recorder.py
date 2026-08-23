#!/usr/bin/env python3
"""Record a vehicle-only motion/perception test from explicitly labelled sources."""

import csv
import math
import os

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from yolov11_ros_msgs.msg import BoundingBoxes


FIELDS = [
    "run_id", "target_speed", "t", "x_gt", "y_gt", "z_gt", "vx_gt",
    "vy_gt", "vz_gt", "speed_gt", "roll_gt", "pitch_gt", "yaw_gt",
    "angular_speed_gt", "tag_visible", "tag_x_map_visual", "tag_y_map_visual",
    "tag_error_x_vs_gt", "tag_error_y_vs_gt", "yolo_detected",
    "yolo_confidence", "yolo_center_x", "yolo_center_y",
]


def quaternion_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


class VehicleMotionRecorder:
    def __init__(self):
        self.run_id = rospy.get_param("~run_id", "vehicle_motion_0p5")
        self.target_speed = float(rospy.get_param("~target_speed", 0.5))
        self.duration = float(rospy.get_param("~duration", 37.0))
        self.output_file = os.path.abspath(rospy.get_param("~output_file"))
        self.start = None
        self.rows = []
        self.last_record = rospy.Time(0)
        self.yolo = (False, "", "", "")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_cb, queue_size=1)
        rospy.Subscriber("/landing/yolo_boxes", BoundingBoxes, self.yolo_cb, queue_size=1)

    def yolo_cb(self, msg):
        vehicles = [box for box in msg.bounding_boxes
                    if box.Class.lower() in ("car", "truck", "bus", "vehicle")]
        if not vehicles:
            self.yolo = (False, "", "", "")
            return
        best = max(vehicles, key=lambda box: box.probability)
        self.yolo = (True, float(best.probability),
                     0.5 * (best.xmin + best.xmax), 0.5 * (best.ymin + best.ymax))

    def models_cb(self, msg):
        now = rospy.Time.now()
        try:
            index = msg.name.index("ugv_0")
        except ValueError:
            return
        if self.start is None:
            self.start = now
        if now - self.last_record < rospy.Duration(0.05):
            return
        self.last_record = now
        pose, twist = msg.pose[index], msg.twist[index]
        roll, pitch, yaw = quaternion_to_rpy(pose.orientation)
        row = dict.fromkeys(FIELDS, "")
        row.update(
            run_id=self.run_id, target_speed=self.target_speed,
            t=(now - self.start).to_sec(), x_gt=pose.position.x,
            y_gt=pose.position.y, z_gt=pose.position.z,
            vx_gt=twist.linear.x, vy_gt=twist.linear.y, vz_gt=twist.linear.z,
            speed_gt=math.hypot(twist.linear.x, twist.linear.y),
            roll_gt=roll, pitch_gt=pitch, yaw_gt=yaw,
            angular_speed_gt=math.sqrt(twist.angular.x ** 2 + twist.angular.y ** 2 +
                                       twist.angular.z ** 2),
            yolo_detected=self.yolo[0], yolo_confidence=self.yolo[1],
            yolo_center_x=self.yolo[2], yolo_center_y=self.yolo[3])
        try:
            tfm = self.tf_buffer.lookup_transform("map", "landing_pad", rospy.Time(0),
                                                  rospy.Duration(0.005))
            if now - tfm.header.stamp <= rospy.Duration(0.6):
                tx = tfm.transform.translation.x
                ty = tfm.transform.translation.y
                row.update(tag_visible=True, tag_x_map_visual=tx, tag_y_map_visual=ty,
                           tag_error_x_vs_gt=tx - pose.position.x,
                           tag_error_y_vs_gt=ty - pose.position.y)
            else:
                row["tag_visible"] = False
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            row["tag_visible"] = False
        self.rows.append(row)
        if row["t"] >= self.duration:
            self.finish()

    def finish(self):
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        rospy.loginfo("vehicle motion test recorded %d samples to %s",
                      len(self.rows), self.output_file)
        rospy.signal_shutdown("vehicle motion recording complete")


if __name__ == "__main__":
    rospy.init_node("vehicle_motion_recorder")
    VehicleMotionRecorder()
    rospy.spin()
