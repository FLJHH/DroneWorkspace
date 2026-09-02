#!/usr/bin/env python3
"""PX4 SITL 50 m glide landing with continuous YOLO/AprilTag fusion."""

import csv
import math
import os

import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from std_msgs.msg import Float32, String
from yolov11_ros_msgs.msg import BoundingBoxes


FIELDS = (
    "t", "phase", "clearance", "horizontal_error_gt", "tag_valid",
    "tag_age", "tag_weight", "yolo_visible", "yolo_confidence",
    "coarse_vx", "coarse_vy", "tag_vx", "tag_vy", "cmd_vx", "cmd_vy",
    "cmd_vz", "uav_x", "uav_y", "uav_z", "vehicle_x", "vehicle_y",
    "mode", "armed",
)


class FusedGlideLanding:
    VEHICLE_CLASSES = {"car", "truck", "bus", "vehicle"}

    def __init__(self):
        self.cruise_clearance = float(rospy.get_param("~cruise_clearance", 50.0))
        self.platform_height = float(rospy.get_param("~platform_height", 2.23))
        self.descent_speed = float(rospy.get_param("~descent_speed", 0.8))
        self.kp_takeoff = float(rospy.get_param("~kp_takeoff", 0.7))
        self.takeoff_max_vz = float(rospy.get_param("~takeoff_max_vz", 1.2))
        self.kp_glide = float(rospy.get_param("~kp_glide", 1.2))
        self.max_xy_speed = float(rospy.get_param("~max_xy_speed", 2.5))
        self.glide_tolerance = float(rospy.get_param("~glide_tolerance", 1.5))
        self.kp_yolo = float(rospy.get_param("~kp_yolo", 0.4))
        self.image_width = float(rospy.get_param("~image_width", 1280.0))
        self.image_height = float(rospy.get_param("~image_height", 720.0))
        self.tag_frame = rospy.get_param("~tag_frame", "landing_pad")
        self.tag_filter_alpha = float(rospy.get_param("~tag_filter_alpha", 0.25))
        self.tag_max_jump = float(rospy.get_param("~tag_max_jump", 0.75))
        self.coarse_drift_std = float(rospy.get_param("~coarse_drift_std", 0.0))
        self.height_noise_std = float(rospy.get_param("~height_noise_std", 0.0))
        self.output_file = os.path.abspath(rospy.get_param("~output_file"))
        self.state = State()
        self.pose = None
        self.extended = ExtendedState()
        self.vehicle_world = None
        self.iris_world = None
        self.vehicle_velocity = (0.0, 0.0)
        self.last_vehicle = rospy.Time(0)
        self.yolo_error = None
        self.yolo_confidence = 0.0
        self.tag_world = None
        self.last_tag_update = rospy.Time(0)
        self.tag_weight = 0.0
        self.phase = "WAIT_FCU"
        # The node can start before Gazebo publishes /clock. Start the timeout
        # from the first valid simulated-time control cycle instead.
        self.phase_since = None
        self.start_time = None
        self.rows = []
        self.completed = False
        self.last_cmd = (0.0,) * 7

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.setpoint_pub = rospy.Publisher(
            "/iris_0/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.phase_pub = rospy.Publisher("/landing/phase", String, queue_size=1,
                                         latch=True)
        self.weight_pub = rospy.Publisher("/landing/fusion_weight", Float32,
                                          queue_size=1, latch=True)
        rospy.Subscriber("/iris_0/mavros/state", State, self.state_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped,
                         self.pose_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/extended_state", ExtendedState,
                         self.extended_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_cb, queue_size=1)
        rospy.Subscriber("/landing/yolo_boxes", BoundingBoxes, self.yolo_cb, queue_size=1)
        self.arm = rospy.ServiceProxy("/iris_0/mavros/cmd/arming", CommandBool)
        self.command_long = rospy.ServiceProxy("/iris_0/mavros/cmd/command", CommandLong)
        self.set_mode = rospy.ServiceProxy("/iris_0/mavros/set_mode", SetMode)
        self.phase_pub.publish(String(data=self.phase))
        self.weight_pub.publish(Float32(data=0.0))
        rospy.on_shutdown(self.save)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def state_cb(self, message):
        self.state = message

    def pose_cb(self, message):
        self.pose = message

    def extended_cb(self, message):
        self.extended = message

    def models_cb(self, message):
        try:
            vehicle_index = message.name.index("ugv_0")
            iris_index = message.name.index("iris_0")
            self.vehicle_world = message.pose[vehicle_index].position
            self.iris_world = message.pose[iris_index].position
            velocity = message.twist[vehicle_index].linear
            self.vehicle_velocity = (velocity.x, velocity.y)
        except ValueError:
            pass

    def yolo_cb(self, message):
        now = rospy.Time.now()
        candidates = [box for box in message.bounding_boxes
                      if box.Class.lower() in self.VEHICLE_CLASSES]
        if not candidates:
            return
        best = max(candidates, key=lambda box: box.probability)
        self.last_vehicle = now
        self.yolo_confidence = float(best.probability)
        cx = 0.5 * (best.xmin + best.xmax)
        cy = 0.5 * (best.ymin + best.ymax)
        self.yolo_error = ((cx - 0.5 * self.image_width) / (0.5 * self.image_width),
                           (cy - 0.5 * self.image_height) / (0.5 * self.image_height))

    def set_phase(self, phase, reason):
        if phase == self.phase:
            return
        rospy.loginfo("fused landing: %s -> %s reason=%s", self.phase, phase, reason)
        self.phase = phase
        self.phase_since = rospy.Time.now()
        self.phase_pub.publish(String(data=phase))

    def publish_velocity(self, vx, vy, vz):
        message = PositionTarget()
        message.header.stamp = rospy.Time.now()
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                             PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                             PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                             PositionTarget.FORCE | PositionTarget.IGNORE_YAW |
                             PositionTarget.IGNORE_YAW_RATE)
        message.velocity.x, message.velocity.y, message.velocity.z = vx, vy, vz
        self.setpoint_pub.publish(message)

    def request_offboard(self):
        try:
            if self.state.mode != "OFFBOARD":
                self.set_mode(base_mode=0, custom_mode="OFFBOARD")
            elif not self.state.armed:
                self.arm(True)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "PX4 service call failed: %s", exc)

    def vehicle_local_xy(self):
        if self.pose is None or self.vehicle_world is None or self.iris_world is None:
            return None
        p = self.pose.pose.position
        origin_x = self.iris_world.x - p.x
        origin_y = self.iris_world.y - p.y
        target_x = self.vehicle_world.x - origin_x
        target_y = self.vehicle_world.y - origin_y
        if self.coarse_drift_std > 0.0:
            t = rospy.Time.now().to_sec()
            amplitude = math.sqrt(2.0) * self.coarse_drift_std
            target_x += amplitude * math.sin(0.071 * t + 0.4)
            target_y += amplitude * math.sin(0.053 * t + 2.1)
        return target_x, target_y

    def update_tag(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", self.tag_frame, rospy.Time(0), rospy.Duration(0.02))
            if rospy.Time.now() - transform.header.stamp > rospy.Duration(0.3):
                return False
            measured = (transform.transform.translation.x,
                        transform.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        if self.tag_world is not None:
            jump = math.hypot(measured[0] - self.tag_world[0],
                              measured[1] - self.tag_world[1])
            if jump > self.tag_max_jump:
                return False
            alpha = self.tag_filter_alpha
            measured = ((1.0 - alpha) * self.tag_world[0] + alpha * measured[0],
                        (1.0 - alpha) * self.tag_world[1] + alpha * measured[1])
        self.tag_world = measured
        self.last_tag_update = rospy.Time.now()
        return True

    @staticmethod
    def height_weight_limit(clearance):
        if clearance >= 40.0:
            return 0.0
        if clearance >= 35.0:
            return 0.30 * (40.0 - clearance) / 5.0
        if clearance >= 30.0:
            return 0.30 + 0.50 * (35.0 - clearance) / 5.0
        return 1.0

    def coarse_velocity(self, target, clearance, p):
        desired_x = target[0] - max(0.0, clearance)
        desired_y = target[1]
        ex, ey = desired_x - p.x, desired_y - p.y
        vx = self.vehicle_velocity[0] + self.descent_speed + self.kp_glide * ex
        vy = self.vehicle_velocity[1] + self.kp_glide * ey
        if (rospy.Time.now() - self.last_vehicle < rospy.Duration(1.0)
                and self.yolo_error is not None):
            image_x, image_y = self.yolo_error
            vx += self.kp_yolo * image_y
            vy -= self.kp_yolo * image_x
        return (self.clamp(vx, self.max_xy_speed),
                self.clamp(vy, self.max_xy_speed), math.hypot(ex, ey))

    def tag_velocity(self, clearance, p, fallback):
        if self.tag_world is None:
            return fallback[0], fallback[1]
        ex, ey = self.tag_world[0] - p.x, self.tag_world[1] - p.y
        remaining = max(0.35, clearance / max(0.05, self.descent_speed))
        return (self.clamp(ex / remaining, self.max_xy_speed),
                self.clamp(ey / remaining, self.max_xy_speed))

    def finish_touchdown(self, target, p):
        if self.iris_world is None or self.iris_world.z >= self.platform_height + 0.37:
            return False
        if math.hypot(target[0] - p.x, target[1] - p.y) >= 0.35:
            return False
        self.publish_velocity(0.0, 0.0, 0.0)
        try:
            response = self.command_long(
                broadcast=False, command=400, confirmation=0,
                param1=0.0, param2=21196.0, param3=0.0, param4=0.0,
                param5=0.0, param6=0.0, param7=0.0)
            if response.success:
                self.completed = True
                self.set_phase("TOUCHDOWN", "roof contact and motor lock")
                self.save()
                return True
        except rospy.ServiceException as exc:
            rospy.logwarn("touchdown disarm failed: %s", exc)
        return False

    def record(self, clearance, target, tag_valid):
        if self.pose is None or self.vehicle_world is None:
            return
        p = self.pose.pose.position
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now
        cvx, cvy, tvx, tvy, vx, vy, vz = self.last_cmd
        self.rows.append({
            "t": (now - self.start_time).to_sec(), "phase": self.phase,
            "clearance": clearance,
            "horizontal_error_gt": math.hypot(target[0] - p.x, target[1] - p.y),
            "tag_valid": int(tag_valid),
            "tag_age": (now - self.last_tag_update).to_sec(),
            "tag_weight": self.tag_weight,
            "yolo_visible": int(now - self.last_vehicle < rospy.Duration(1.0)),
            "yolo_confidence": self.yolo_confidence,
            "coarse_vx": cvx, "coarse_vy": cvy, "tag_vx": tvx, "tag_vy": tvy,
            "cmd_vx": vx, "cmd_vy": vy, "cmd_vz": vz,
            "uav_x": p.x, "uav_y": p.y, "uav_z": p.z,
            "vehicle_x": self.vehicle_world.x, "vehicle_y": self.vehicle_world.y,
            "mode": self.state.mode, "armed": int(self.state.armed),
        })

    def save(self):
        if not self.rows:
            return
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)

    def update(self):
        if not self.state.connected or self.pose is None:
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        if self.phase == "WAIT_FCU":
            self.publish_velocity(0.0, 0.0, 0.0)
            if self.phase_since is None:
                self.phase_since = rospy.Time.now()
            if rospy.Time.now() - self.phase_since > rospy.Duration(2.5):
                self.set_phase("TAKEOFF", "startup delay complete")
            return
        if self.phase != "TOUCHDOWN":
            self.request_offboard()
        p = self.pose.pose.position
        target = self.vehicle_local_xy()
        if target is None or self.iris_world is None:
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        clearance = self.iris_world.z - self.platform_height
        if self.height_noise_std > 0.0:
            clearance += self.height_noise_std * math.sin(
                1.73 * rospy.Time.now().to_sec() + 0.8)
        tag_valid = self.update_tag()
        if self.phase == "TAKEOFF":
            vz = self.clamp(self.kp_takeoff * (self.cruise_clearance - clearance),
                            self.takeoff_max_vz)
            self.publish_velocity(0.0, 0.0, vz)
            self.last_cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, vz)
            if clearance >= self.cruise_clearance - 0.25:
                self.set_phase(
                    "GLIDE_APPROACH",
                    "{:.0f} m clearance reached".format(self.cruise_clearance))
        elif self.phase == "GLIDE_APPROACH":
            if self.finish_touchdown(target, p):
                return
            coarse_vx, coarse_vy, path_error = self.coarse_velocity(target, clearance, p)
            tag_vx, tag_vy = self.tag_velocity(clearance, p, (coarse_vx, coarse_vy))
            if tag_valid:
                self.tag_weight += 0.05
            else:
                self.tag_weight -= 0.10
            self.tag_weight = max(0.0, min(
                self.tag_weight, self.height_weight_limit(clearance)))
            vx = ((1.0 - self.tag_weight) * coarse_vx + self.tag_weight * tag_vx)
            vy = ((1.0 - self.tag_weight) * coarse_vy + self.tag_weight * tag_vy)
            vz = -self.descent_speed if path_error <= self.glide_tolerance else 0.0
            self.publish_velocity(vx, vy, vz)
            self.weight_pub.publish(Float32(data=self.tag_weight))
            self.last_cmd = (coarse_vx, coarse_vy, tag_vx, tag_vy, vx, vy, vz)
            rospy.loginfo_throttle(
                0.5, "fused glide clearance=%.2f path=%.2f tag=%s weight=%.2f "
                "velocity=(%.2f,%.2f,%.2f)", clearance, path_error, tag_valid,
                self.tag_weight, vx, vy, vz)
        elif self.phase == "TOUCHDOWN":
            self.publish_velocity(0.0, 0.0, 0.0)
        self.record(clearance, target, tag_valid)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.update()
            if self.completed:
                rospy.sleep(1.0)
                rospy.signal_shutdown("fused landing complete")
                return
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fused_glide_landing")
    FusedGlideLanding().run()
