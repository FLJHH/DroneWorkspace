#!/usr/bin/env python3
"""Record landing/vehicle experiments without influencing flight control."""

import csv
import json
import math
import os
from datetime import datetime

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import String
from yolov11_ros_msgs.msg import BoundingBoxes


SUMMARY_FIELDS = [
    "run_id", "timestamp", "mode", "vehicle_speed", "success",
    "takeoff_time", "tag_handoff_time", "touchdown_time", "total_time",
    "max_horizontal_error_gt", "tag_loss_count", "max_tag_loss_duration",
    "handoff_horizontal_error_visual", "handoff_horizontal_error_gt",
    "handoff_clearance_gt", "touchdown_horizontal_error_gt",
    "uav_vx_touchdown_gt", "uav_vy_touchdown_gt", "uav_vz_touchdown_gt",
    "uav_vx_touchdown_mavros", "uav_vy_touchdown_mavros",
    "uav_vz_touchdown_mavros", "vehicle_vx_touchdown_gt",
    "vehicle_vy_touchdown_gt", "relative_speed_touchdown_gt",
    "offboard_lost", "disarm_success", "possible_bounce",
    "bounce_height", "failure_reason",
]

SAMPLE_FIELDS = [
    "t", "phase", "tag_visible", "uav_x_gt", "uav_y_gt", "uav_z_gt",
    "uav_vx_gt", "uav_vy_gt", "uav_vz_gt", "uav_roll_gt",
    "uav_pitch_gt", "uav_yaw_gt", "vehicle_x_gt", "vehicle_y_gt",
    "vehicle_z_gt", "vehicle_vx_gt", "vehicle_vy_gt", "vehicle_vz_gt",
    "vehicle_speed_gt", "vehicle_yaw_gt", "horizontal_error_gt",
    "uav_x_mavros", "uav_y_mavros", "uav_z_mavros", "uav_vx_mavros",
    "uav_vy_mavros", "uav_vz_mavros", "tag_x_map_visual",
    "tag_y_map_visual", "tag_error_visual", "yolo_detected",
    "yolo_confidence", "yolo_center_x", "yolo_center_y", "armed",
    "flight_mode",
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


class ExperimentRecorder:
    def __init__(self):
        self.mode = rospy.get_param("~mode", "static_baseline")
        self.run_id = rospy.get_param("~run_id", datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.vehicle_speed = float(rospy.get_param("~vehicle_speed", 0.0))
        self.output_dir = os.path.abspath(rospy.get_param("~output_dir"))
        self.max_duration = float(rospy.get_param("~max_duration", 150.0))
        self.post_touchdown_duration = float(rospy.get_param("~post_touchdown_duration", 2.0))
        self.platform_height = float(rospy.get_param("~platform_height", 2.23))
        for child in ("logs", "plots"):
            os.makedirs(os.path.join(self.output_dir, child), exist_ok=True)

        self.start = None
        self.phase = "UNKNOWN"
        self.phase_times = {}
        self.state = State()
        self.seen_offboard = False
        self.offboard_lost = False
        self.model_msg = None
        self.local_pose = None
        self.local_velocity = None
        self.tag_xy = None
        self.tag_stamp = rospy.Time(0)
        self.yolo = (False, "", "", "")
        self.samples = []
        self.last_sample = rospy.Time(0)
        self.tag_loss_count = 0
        self.loss_started = None
        self.loss_durations = []
        self.max_horizontal_error = 0.0
        self.handoff = None
        self.touchdown = None
        self.touchdown_z = None
        self.disarm_success = False
        self.finished = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/landing/phase", String, self.phase_cb, queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/state", State, self.state_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/local_position/velocity_local", TwistStamped,
                         self.velocity_cb, queue_size=1)
        rospy.Subscriber("/landing/yolo_boxes", BoundingBoxes, self.yolo_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.timer_cb)

    def elapsed(self, now=None):
        now = now or rospy.Time.now()
        return (now - self.start).to_sec() if self.start is not None else 0.0

    def phase_cb(self, msg):
        now = rospy.Time.now()
        if self.start is None:
            self.start = now
        previous = self.phase
        self.phase = msg.data
        self.phase_times.setdefault(self.phase, self.elapsed(now))
        if self.phase == "TAKEOFF":
            self.phase_times["TAKEOFF"] = self.elapsed(now)
        if self.phase == "TAG_APPROACH" and previous != "TAG_DEAD_RECKON" and self.handoff is None:
            self.handoff = self.current_metrics()
        if self.phase == "TAG_DEAD_RECKON" and previous != "TAG_DEAD_RECKON":
            self.tag_loss_count += 1
            self.loss_started = now
        elif previous == "TAG_DEAD_RECKON" and self.loss_started is not None:
            self.loss_durations.append((now - self.loss_started).to_sec())
            self.loss_started = None
        if self.phase == "DONE" and self.touchdown is None:
            self.touchdown = self.current_metrics()
            self.touchdown["time"] = self.elapsed(now)
            self.touchdown_z = self.touchdown.get("uav_z_gt")

    def state_cb(self, msg):
        if msg.mode == "OFFBOARD":
            self.seen_offboard = True
        elif self.seen_offboard and self.state.armed and self.phase not in ("DONE", "AUTO_LAND"):
            self.offboard_lost = True
        self.state = msg
        if self.touchdown is not None and not msg.armed:
            self.disarm_success = True

    def pose_cb(self, msg):
        self.local_pose = msg

    def velocity_cb(self, msg):
        self.local_velocity = msg

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
        self.model_msg = msg

    def model_state(self, name):
        if self.model_msg is None:
            return None, None
        try:
            index = self.model_msg.name.index(name)
        except ValueError:
            return None, None
        return self.model_msg.pose[index], self.model_msg.twist[index]

    def update_tag(self, now):
        try:
            tfm = self.tf_buffer.lookup_transform("map", "landing_pad", rospy.Time(0),
                                                  rospy.Duration(0.01))
            if now - tfm.header.stamp <= rospy.Duration(0.6):
                self.tag_xy = (tfm.transform.translation.x, tfm.transform.translation.y)
                self.tag_stamp = tfm.header.stamp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    def current_metrics(self):
        iris_pose, iris_twist = self.model_state("iris_0")
        vehicle_pose, vehicle_twist = self.model_state("ugv_0")
        result = {}
        if iris_pose is not None:
            result.update(uav_x_gt=iris_pose.position.x, uav_y_gt=iris_pose.position.y,
                          uav_z_gt=iris_pose.position.z,
                          uav_vx_gt=iris_twist.linear.x, uav_vy_gt=iris_twist.linear.y,
                          uav_vz_gt=iris_twist.linear.z)
        if vehicle_pose is not None:
            result.update(vehicle_x_gt=vehicle_pose.position.x,
                          vehicle_y_gt=vehicle_pose.position.y,
                          vehicle_z_gt=vehicle_pose.position.z,
                          vehicle_vx_gt=vehicle_twist.linear.x,
                          vehicle_vy_gt=vehicle_twist.linear.y,
                          vehicle_vz_gt=vehicle_twist.linear.z)
        if iris_pose is not None and vehicle_pose is not None:
            result["horizontal_error_gt"] = math.hypot(
                iris_pose.position.x - vehicle_pose.position.x,
                iris_pose.position.y - vehicle_pose.position.y)
            result["clearance_gt"] = iris_pose.position.z - self.platform_height
            result["relative_speed_gt"] = math.hypot(
                iris_twist.linear.x - vehicle_twist.linear.x,
                iris_twist.linear.y - vehicle_twist.linear.y)
        if self.local_pose is not None:
            p = self.local_pose.pose.position
            result.update(uav_x_mavros=p.x, uav_y_mavros=p.y, uav_z_mavros=p.z)
            if self.tag_xy is not None:
                result["tag_error_visual"] = math.hypot(self.tag_xy[0] - p.x,
                                                         self.tag_xy[1] - p.y)
        if self.local_velocity is not None:
            v = self.local_velocity.twist.linear
            result.update(uav_vx_mavros=v.x, uav_vy_mavros=v.y, uav_vz_mavros=v.z)
        return result

    def timer_cb(self, _event):
        if self.finished:
            return
        now = rospy.Time.now()
        if self.start is None or now == rospy.Time(0):
            return
        self.update_tag(now)
        if now - self.last_sample >= rospy.Duration(0.05):
            self.record_sample(now)
            self.last_sample = now
        if self.touchdown is not None and self.elapsed(now) >= self.touchdown["time"] + self.post_touchdown_duration:
            self.finalize(success=self.disarm_success, reason="" if self.disarm_success else "disarm_not_confirmed")
        elif self.elapsed(now) >= self.max_duration:
            self.finalize(success=False, reason="experiment_timeout")

    def record_sample(self, now):
        values = {field: "" for field in SAMPLE_FIELDS}
        values.update(t=self.elapsed(now), phase=self.phase,
                      tag_visible=(now - self.tag_stamp <= rospy.Duration(0.6)),
                      armed=self.state.armed, flight_mode=self.state.mode)
        values.update(self.current_metrics())
        iris_pose, _ = self.model_state("iris_0")
        vehicle_pose, vehicle_twist = self.model_state("ugv_0")
        if iris_pose is not None:
            values["uav_roll_gt"], values["uav_pitch_gt"], values["uav_yaw_gt"] = quaternion_to_rpy(iris_pose.orientation)
        if vehicle_pose is not None:
            values["vehicle_yaw_gt"] = quaternion_to_rpy(vehicle_pose.orientation)[2]
            values["vehicle_speed_gt"] = math.hypot(vehicle_twist.linear.x,
                                                     vehicle_twist.linear.y)
        if self.tag_xy is not None:
            values["tag_x_map_visual"], values["tag_y_map_visual"] = self.tag_xy
        values["yolo_detected"], values["yolo_confidence"], values["yolo_center_x"], values["yolo_center_y"] = self.yolo
        if self.phase in ("TAG_APPROACH", "TAG_DEAD_RECKON") and values.get("horizontal_error_gt") != "":
            self.max_horizontal_error = max(self.max_horizontal_error,
                                            values["horizontal_error_gt"])
        self.samples.append(values)

    def finalize(self, success, reason):
        now = rospy.Time.now()
        if self.loss_started is not None:
            self.loss_durations.append((now - self.loss_started).to_sec())
        sample_path = os.path.join(self.output_dir, "logs", self.run_id + ".csv")
        with open(sample_path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=SAMPLE_FIELDS)
            writer.writeheader()
            writer.writerows({field: row.get(field, "") for field in SAMPLE_FIELDS}
                             for row in self.samples)

        handoff = self.handoff or {}
        touchdown = self.touchdown or {}
        takeoff_time = self.phase_times.get("TAKEOFF", "")
        touchdown_time = touchdown.get("time", "")
        first_contact_z = self.platform_height + 0.37
        post = [row for row in self.samples
                if touchdown_time != "" and row["t"] >= touchdown_time]
        max_post_z = max((float(row["uav_z_gt"]) for row in post
                          if row["uav_z_gt"] != ""), default=float("nan"))
        bounce_height = (max_post_z - first_contact_z
                         if not math.isnan(max_post_z) else "")
        possible_bounce = (bounce_height != "" and bounce_height > 0.10)
        summary = dict.fromkeys(SUMMARY_FIELDS, "")
        summary.update(
            run_id=self.run_id, timestamp=datetime.now().isoformat(timespec="seconds"),
            mode=self.mode, vehicle_speed=self.vehicle_speed, success=success,
            takeoff_time=takeoff_time,
            tag_handoff_time=self.phase_times.get("TAG_APPROACH", ""),
            touchdown_time=touchdown_time,
            total_time=(touchdown_time - takeoff_time
                        if touchdown_time != "" and takeoff_time != "" else ""),
            max_horizontal_error_gt=self.max_horizontal_error,
            tag_loss_count=self.tag_loss_count,
            max_tag_loss_duration=max(self.loss_durations, default=0.0),
            handoff_horizontal_error_visual=handoff.get("tag_error_visual", ""),
            handoff_horizontal_error_gt=handoff.get("horizontal_error_gt", ""),
            handoff_clearance_gt=handoff.get("clearance_gt", ""),
            touchdown_horizontal_error_gt=touchdown.get("horizontal_error_gt", ""),
            uav_vx_touchdown_gt=touchdown.get("uav_vx_gt", ""),
            uav_vy_touchdown_gt=touchdown.get("uav_vy_gt", ""),
            uav_vz_touchdown_gt=touchdown.get("uav_vz_gt", ""),
            uav_vx_touchdown_mavros=touchdown.get("uav_vx_mavros", ""),
            uav_vy_touchdown_mavros=touchdown.get("uav_vy_mavros", ""),
            uav_vz_touchdown_mavros=touchdown.get("uav_vz_mavros", ""),
            vehicle_vx_touchdown_gt=touchdown.get("vehicle_vx_gt", ""),
            vehicle_vy_touchdown_gt=touchdown.get("vehicle_vy_gt", ""),
            relative_speed_touchdown_gt=touchdown.get("relative_speed_gt", ""),
            offboard_lost=self.offboard_lost, disarm_success=self.disarm_success,
            possible_bounce=possible_bounce, bounce_height=bounce_height,
            failure_reason=reason)
        summary_path = os.path.join(self.output_dir, "static_baseline.csv")
        write_header = not os.path.exists(summary_path) or os.path.getsize(summary_path) == 0
        with open(summary_path, "a", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
            if write_header:
                writer.writeheader()
            writer.writerow(summary)
        with open(os.path.join(self.output_dir, "logs", self.run_id + ".json"), "w") as stream:
            json.dump({"summary": summary, "sources": {
                "visual": ["tag_x_map_visual", "tag_y_map_visual", "tag_error_visual",
                           "yolo_detected", "yolo_confidence", "yolo_center_x", "yolo_center_y"],
                "mavros": ["uav_x_mavros", "uav_y_mavros", "uav_z_mavros",
                           "uav_vx_mavros", "uav_vy_mavros", "uav_vz_mavros",
                           "armed", "flight_mode"],
                "gazebo_ground_truth": [field for field in SAMPLE_FIELDS if field.endswith("_gt")],
            }}, stream, indent=2)
        rospy.loginfo("experiment %s finalized: success=%s summary=%s", self.run_id,
                      success, summary_path)
        self.finished = True
        rospy.signal_shutdown("experiment complete")


if __name__ == "__main__":
    rospy.init_node("experiment_recorder")
    ExperimentRecorder()
    rospy.spin()
