#!/usr/bin/env python3
"""YOLO-gated, AprilTag-guided precision landing controller for PX4 SITL."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from std_msgs.msg import String
from yolov11_ros_msgs.msg import BoundingBoxes


class VehicleLanding(object):
    VEHICLE_CLASSES = {"car", "truck", "bus", "vehicle"}

    def __init__(self):
        self.auto_start = rospy.get_param("~auto_start", True)
        self.staging_x = rospy.get_param("~staging_x", 8.0)
        self.staging_y = rospy.get_param("~staging_y", -2.2)
        self.cruise_height = rospy.get_param("~cruise_height", 6.0)
        self.tag_frame = rospy.get_param("~tag_frame", "landing_pad")
        self.vehicle_timeout = rospy.Duration(rospy.get_param("~vehicle_timeout", 1.0))
        self.person_pause = rospy.get_param("~person_pause", True)
        self.kp_transit = rospy.get_param("~kp_transit", 0.7)
        self.kp_glide = rospy.get_param("~kp_glide", 2.0)
        self.kp_align = rospy.get_param("~kp_align", 0.8)
        self.max_xy_speed = rospy.get_param("~max_xy_speed", 2.5)
        self.descent_speed = rospy.get_param("~descent_speed", 0.80)
        self.align_tolerance = rospy.get_param("~align_tolerance", 0.18)
        # Permit a coupled approach while the tag is well below the aircraft.
        # The admissible horizontal error shrinks with range, so the final
        # segment still requires precise centring instead of hovering forever
        # at the handoff altitude waiting for a fixed 18 cm error.
        # The camera is intentionally tilted for the oblique YOLO approach;
        # therefore a visible tag can legitimately have horizontal/vertical
        # translation greater than one near the roof.  A 1.5 ratio corresponds
        # to roughly 56 degrees and keeps descent active while it remains in
        # the useful image region.
        self.descent_cone_ratio = rospy.get_param("~descent_cone_ratio", 1.5)
        self.land_tag_distance = rospy.get_param("~land_tag_distance", 0.55)
        self.image_width = float(rospy.get_param("~image_width", 640))
        self.image_height = float(rospy.get_param("~image_height", 480))
        self.kp_yolo = rospy.get_param("~kp_yolo", 0.9)
        self.yolo_deadband = rospy.get_param("~yolo_deadband", 0.08)
        self.vehicle_velocity = [rospy.get_param("~vehicle_speed", 0.35), 0.0]
        self.landing_lead_time = rospy.get_param("~landing_lead_time", 0.0)
        self.landing_forward_offset = rospy.get_param(
            "~landing_forward_offset", 0.35)
        self.final_align_height = rospy.get_param("~final_align_height", 1.2)
        self.final_align_tolerance = rospy.get_param(
            "~final_align_tolerance", 0.20)
        self.yolo_error = None
        self.vehicle_confirmed = False
        self.demo_ground_truth = rospy.get_param("~demo_ground_truth", True)
        self.observation_back = rospy.get_param("~observation_back", 8.0)
        self.observation_side = rospy.get_param("~observation_side", -3.0)
        self.platform_height = rospy.get_param("~platform_height", 2.23)
        self.glide_tolerance = rospy.get_param("~glide_tolerance", 1.0)
        self.tag_handoff_height = rospy.get_param("~tag_handoff_height", 6.0)
        self.tag_handoff_horizontal = rospy.get_param("~tag_handoff_horizontal", 3.0)
        self.max_tag_correction = rospy.get_param("~max_tag_correction", 0.6)
        self.tag_filter_alpha = rospy.get_param("~tag_filter_alpha", 0.25)
        self.tag_max_jump = rospy.get_param("~tag_max_jump", 0.75)
        self.dead_reckon_timeout = rospy.Duration(
            rospy.get_param("~dead_reckon_timeout", 3.0))
        self.vehicle_world = None
        self.iris_world = None
        self.tag_world = None
        self.last_tag_update = rospy.Time(0)

        self.state = State()
        self.pose = None
        self.extended = ExtendedState()
        self.last_vehicle = rospy.Time(0)
        self.last_person = rospy.Time(0)
        self.phase = "WAIT_FCU"
        self.phase_since = rospy.Time.now()
        self.offboard_requested = False
        self.land_requested = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.setpoint_pub = rospy.Publisher(
            "/iris_0/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        # Diagnostic-only state output. It does not participate in control.
        self.phase_pub = rospy.Publisher("/landing/phase", String, queue_size=1,
                                         latch=True)
        self.phase_pub.publish(String(data=self.phase))
        rospy.Subscriber("/iris_0/mavros/state", State, self.state_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber("/iris_0/mavros/extended_state", ExtendedState, self.extended_cb, queue_size=1)
        rospy.Subscriber("/landing/yolo_boxes", BoundingBoxes, self.yolo_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_cb, queue_size=1)
        self.arm = rospy.ServiceProxy("/iris_0/mavros/cmd/arming", CommandBool)
        self.command_long = rospy.ServiceProxy("/iris_0/mavros/cmd/command", CommandLong)
        self.set_mode = rospy.ServiceProxy("/iris_0/mavros/set_mode", SetMode)

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.pose = msg

    def extended_cb(self, msg):
        self.extended = msg

    def models_cb(self, msg):
        try:
            vehicle_index = msg.name.index("ugv_0")
            self.vehicle_world = msg.pose[vehicle_index].position
            vehicle_twist = msg.twist[vehicle_index].linear
            # The demo vehicle reverses at each end of its short road segment;
            # use its measured ENU velocity instead of assuming +X forever.
            self.vehicle_velocity = [vehicle_twist.x, vehicle_twist.y]
            self.iris_world = msg.pose[msg.name.index("iris_0")].position
        except ValueError:
            pass

    def vehicle_local_xy(self):
        """Convert Gazebo vehicle coordinates into the PX4 local ENU frame."""
        if self.pose is None or self.vehicle_world is None or self.iris_world is None:
            return None
        p = self.pose.pose.position
        origin_x = self.iris_world.x - p.x
        origin_y = self.iris_world.y - p.y
        return self.vehicle_world.x - origin_x, self.vehicle_world.y - origin_y

    def yolo_cb(self, msg):
        now = rospy.Time.now()
        best = None
        for box in msg.bounding_boxes:
            label = box.Class.lower()
            if label in self.VEHICLE_CLASSES and box.probability >= 0.30:
                self.last_vehicle = now
                self.vehicle_confirmed = True
                area = max(0, box.xmax - box.xmin) * max(0, box.ymax - box.ymin)
                if best is None or area > best[0]:
                    cx = 0.5 * (box.xmin + box.xmax)
                    cy = 0.5 * (box.ymin + box.ymax)
                    best = (area, (cx - 0.5 * self.image_width) / (0.5 * self.image_width),
                            (cy - 0.5 * self.image_height) / (0.5 * self.image_height))
            elif label == "person" and box.probability >= 0.35:
                self.last_person = now
        if best is not None:
            self.yolo_error = (best[1], best[2])

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def set_phase(self, phase):
        if phase != self.phase:
            rospy.loginfo("landing state: %s -> %s", self.phase, phase)
            self.phase = phase
            self.phase_since = rospy.Time.now()
            self.phase_pub.publish(String(data=self.phase))

    def publish_enu_velocity(self, vx, vy, vz):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                         PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.FORCE | PositionTarget.IGNORE_YAW |
                         PositionTarget.IGNORE_YAW_RATE)
        # MAVROS converts the ROS ENU velocity fields to PX4 NED.
        msg.velocity.x, msg.velocity.y, msg.velocity.z = vx, vy, vz
        self.setpoint_pub.publish(msg)

    def publish_body_velocity(self, forward, left, up):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                         PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                         PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.FORCE | PositionTarget.IGNORE_YAW |
                         PositionTarget.IGNORE_YAW_RATE)
        # MAVROS accepts ROS base_link FLU here and converts it to PX4 BODY_NED.
        msg.velocity.x, msg.velocity.y, msg.velocity.z = forward, left, up
        self.setpoint_pub.publish(msg)

    def vehicle_visible(self):
        return rospy.Time.now() - self.last_vehicle < self.vehicle_timeout

    def yolo_track_velocity(self):
        """Map downward-camera image error to body-frame planar velocity."""
        if self.yolo_error is None:
            return 0.0, 0.0
        ex, ey = self.yolo_error
        if abs(ex) < self.yolo_deadband:
            ex = 0.0
        if abs(ey) < self.yolo_deadband:
            ey = 0.0
        # Optical right maps to body right (-left); optical down maps forward.
        return self.clamp(self.kp_yolo * ey, 1.0), self.clamp(-self.kp_yolo * ex, 1.0)

    def person_visible(self):
        return rospy.Time.now() - self.last_person < rospy.Duration(0.8)

    def vehicle_body_velocity(self):
        """Rotate Gazebo ENU platform velocity into aircraft body FLU."""
        if self.pose is None:
            return 0.0, 0.0
        q = self.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        vx, vy = self.vehicle_velocity
        return (cos_yaw * vx + sin_yaw * vy,
                -sin_yaw * vx + cos_yaw * vy)

    def body_error_to_enu(self, forward, left):
        """Rotate a body-FLU planar displacement into local ENU."""
        if self.pose is None:
            return 0.0, 0.0
        q = self.pose.pose.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return (cos_yaw * forward - sin_yaw * left,
                sin_yaw * forward + cos_yaw * left)

    def landing_advance_enu(self):
        """Dynamic look-ahead plus calibrated forward roof offset in ENU."""
        vx, vy = self.vehicle_velocity
        speed = math.hypot(vx, vy)
        if speed > 0.05:
            offset_x = self.landing_forward_offset * vx / speed
            offset_y = self.landing_forward_offset * vy / speed
        else:
            offset_x = offset_y = 0.0
        return (vx * self.landing_lead_time + offset_x,
                vy * self.landing_lead_time + offset_y)

    def tag_transform(self):
        try:
            tfm = self.tf_buffer.lookup_transform(
                "iris_0/base_link", self.tag_frame, rospy.Time(0), rospy.Duration(0.03))
            if rospy.Time.now() - tfm.header.stamp > rospy.Duration(0.6):
                return None
            return tfm.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def update_tag_world(self, tag):
        """Fuse a visible tag measurement directly in the MAVROS map frame."""
        if tag is None:
            return False
        try:
            # Let tf2 apply the complete camera extrinsic and aircraft
            # attitude. A yaw-only rotation of the base_link translation is
            # wrong for this 45-degree oblique camera.
            tfm = self.tf_buffer.lookup_transform(
                "map", self.tag_frame, rospy.Time(0), rospy.Duration(0.03))
            if rospy.Time.now() - tfm.header.stamp > rospy.Duration(0.6):
                return False
            measured = (tfm.transform.translation.x,
                        tfm.transform.translation.y)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return False
        if self.tag_world is None:
            self.tag_world = measured
        else:
            jump = math.hypot(measured[0] - self.tag_world[0],
                              measured[1] - self.tag_world[1])
            if jump > self.tag_max_jump:
                rospy.logwarn_throttle(
                    1.0, "rejecting AprilTag world-position jump %.2f m", jump)
                return False
            alpha = self.tag_filter_alpha
            self.tag_world = (
                (1.0 - alpha) * self.tag_world[0] + alpha * measured[0],
                (1.0 - alpha) * self.tag_world[1] + alpha * measured[1])
        self.last_tag_update = rospy.Time.now()
        return True

    def tag_guidance_velocity(self, clearance):
        """Make horizontal error converge at the same time as vertical descent."""
        if self.tag_world is None or self.pose is None:
            return 0.0, 0.0, float("inf")
        p = self.pose.pose.position
        ex = self.tag_world[0] - p.x
        ey = self.tag_world[1] - p.y
        time_to_touchdown = max(0.35, clearance / max(0.05, self.descent_speed))
        vx = self.clamp(ex / time_to_touchdown, self.max_xy_speed)
        vy = self.clamp(ey / time_to_touchdown, self.max_xy_speed)
        return vx, vy, math.hypot(ex, ey)

    def request_flight_mode(self):
        try:
            if self.state.mode != "OFFBOARD":
                if self.set_mode(base_mode=0, custom_mode="OFFBOARD").mode_sent:
                    rospy.loginfo("PX4 OFFBOARD accepted")
            elif not self.state.armed:
                if self.arm(True).success:
                    rospy.loginfo("PX4 armed")
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "PX4 service call failed: %s", exc)

    def request_land(self):
        if self.land_requested:
            return
        try:
            response = self.set_mode(base_mode=0, custom_mode="AUTO.LAND")
            self.land_requested = response.mode_sent
            if self.land_requested:
                self.set_phase("AUTO_LAND")
        except rospy.ServiceException as exc:
            rospy.logwarn("AUTO.LAND request failed: %s", exc)

    def finish_platform_touchdown(self, horizontal_limit=0.35):
        """Lock motors once the estimated airframe height reaches the roof."""
        target = self.tag_world
        if (target is None or self.pose is None or self.iris_world is None or
                self.iris_world.z >= self.platform_height + 0.37):
            return False
        p = self.pose.pose.position
        if math.hypot(target[0] - p.x, target[1] - p.y) >= horizontal_limit:
            return False
        self.publish_enu_velocity(0.0, 0.0, 0.0)
        try:
            response = self.command_long(
                broadcast=False, command=400, confirmation=0,
                param1=0.0, param2=21196.0, param3=0.0,
                param4=0.0, param5=0.0, param6=0.0, param7=0.0)
            if response.success:
                rospy.loginfo("touchdown on moving platform; motors locked")
                self.set_phase("DONE")
                return True
        except rospy.ServiceException as exc:
            rospy.logwarn("disarm at touchdown failed: %s", exc)
        return False

    def update(self):
        if not self.state.connected or self.pose is None:
            self.publish_enu_velocity(0.0, 0.0, 0.0)
            return

        if self.phase == "WAIT_FCU":
            self.publish_enu_velocity(0.0, 0.0, 0.0)
            if self.auto_start and rospy.Time.now() - self.phase_since > rospy.Duration(2.5):
                self.set_phase("TAKEOFF")
            return

        if self.phase not in ("AUTO_LAND", "DONE"):
            self.request_flight_mode()

        p = self.pose.pose.position
        if self.phase == "TAKEOFF":
            vz = self.clamp(self.kp_transit * (self.cruise_height - p.z), 1.2)
            self.publish_enu_velocity(0.0, 0.0, vz)
            if p.z > self.cruise_height - 0.25:
                self.set_phase("TRANSIT")

        elif self.phase == "TRANSIT":
            target = self.vehicle_local_xy() if self.demo_ground_truth else None
            if target is not None:
                # Observe from the moving vehicle's rear quarter. The camera
                # looks forward, exposing the roof, rear and one side to YOLO.
                target_x = target[0] - self.observation_back
                target_y = target[1] + self.observation_side
            else:
                target_x, target_y = self.staging_x, self.staging_y
            ex, ey = target_x - p.x, target_y - p.y
            self.publish_enu_velocity(self.clamp(self.kp_transit * ex, self.max_xy_speed),
                                      self.clamp(self.kp_transit * ey, self.max_xy_speed),
                                      self.clamp(self.kp_transit * (self.cruise_height - p.z), 0.6))
            if math.hypot(ex, ey) < 0.45:
                self.set_phase("ACQUIRE")

        elif self.phase == "ACQUIRE":
            tag = self.tag_transform()
            target = self.vehicle_local_xy()
            if self.demo_ground_truth and target is not None:
                if not self.vehicle_confirmed:
                    observation_x = target[0] - self.observation_back
                    observation_y = target[1] + self.observation_side
                    ex = observation_x - p.x
                    ey = observation_y - p.y
                    self.publish_enu_velocity(
                        self.clamp(self.kp_transit * ex, self.max_xy_speed),
                        self.clamp(self.kp_transit * ey, self.max_xy_speed),
                        self.clamp(self.kp_transit * (self.cruise_height - p.z), 0.5))
                    rospy.logwarn_throttle(
                        1.0, "rear-quarter observation: waiting for YOLO vehicle")
                    return
                rospy.loginfo_throttle(
                    1.0, "YOLO vehicle confirmed; following oblique glide path")

                # Keep the vehicle on the optical axis throughout descent.
                # At cruise height the aircraft is at the rear-quarter
                # observation offset. As height above the roof decreases, the
                # offset contracts proportionally to zero at touchdown.
                world_height = self.iris_world.z if self.iris_world is not None else p.z
                height_above_roof = max(0.0, world_height - self.platform_height)
                actual_horizontal = math.hypot(target[0] - p.x, target[1] - p.y)
                # At close range the full vehicle no longer fits the detector's
                # ideal scale. Once the marker is available, hand control to a
                # pure AprilTag loop and stop depending on YOLO confidence.
                if (height_above_roof <= self.tag_handoff_height and
                        actual_horizontal <= self.tag_handoff_horizontal and
                        tag is not None):
                    self.update_tag_world(tag)
                    rospy.loginfo(
                        "AprilTag acquired at %.2f m, horizontal %.2f m; "
                        "YOLO guidance released",
                        height_above_roof, actual_horizontal)
                    self.publish_body_velocity(0.0, 0.0, 0.0)
                    self.set_phase("TAG_APPROACH")
                    return
                cruise_above_roof = max(1.0, self.cruise_height - self.platform_height)
                glide_fraction = min(1.0, height_above_roof / cruise_above_roof)
                # Aim slightly ahead of the moving roof to compensate for
                # horizontal response and contact latency during fast descent.
                lead_x, lead_y = self.landing_advance_enu()
                desired_x = (target[0] + lead_x -
                             self.observation_back * glide_fraction)
                desired_y = (target[1] + lead_y +
                             self.observation_side * glide_fraction)
                ex, ey = desired_x - p.x, desired_y - p.y
                glide_error = math.hypot(ex, ey)
                if self.person_pause and self.person_visible():
                    vz = 0.0
                elif (height_above_roof < self.final_align_height and
                      glide_error > self.final_align_tolerance):
                    vz = 0.0
                else:
                    vz = -self.descent_speed if glide_error < self.glide_tolerance else 0.0
                # Feed forward both platform motion and contraction of the
                # oblique observation offset. Without this term a P-only loop
                # retains a large steady error against the fast moving target.
                commanded_down = max(0.0, -vz)
                path_vx = (self.vehicle_velocity[0] +
                           self.observation_back * commanded_down /
                           cruise_above_roof)
                path_vy = (self.vehicle_velocity[1] -
                           self.observation_side * commanded_down /
                           cruise_above_roof)
                self.publish_enu_velocity(
                    self.clamp(path_vx + self.kp_glide * ex, self.max_xy_speed),
                    self.clamp(path_vy + self.kp_glide * ey, self.max_xy_speed), vz)
                rospy.loginfo_throttle(
                    0.5, "glide path error x=%.2f y=%.2f roof=%.2f offset=%.2f visible=%s",
                    ex, ey, height_above_roof, actual_horizontal,
                    self.vehicle_visible())
                # Keep matching the moving platform all the way to contact.
                # AUTO.LAND would discard these horizontal velocity commands,
                # allowing the vehicle to drive out from under the aircraft.
                if (actual_horizontal < 0.45 and self.iris_world is not None and
                        self.iris_world.z < 2.60):
                    self.publish_enu_velocity(0.0, 0.0, 0.0)
                    try:
                        response = self.command_long(
                            broadcast=False, command=400, confirmation=0,
                            param1=0.0, param2=21196.0, param3=0.0,
                            param4=0.0, param5=0.0, param6=0.0, param7=0.0)
                        if response.success:
                            rospy.loginfo("touchdown on moving platform; motors locked")
                            self.set_phase("DONE")
                    except rospy.ServiceException as exc:
                        rospy.logwarn("disarm at touchdown failed: %s", exc)
                return
            if self.vehicle_visible():
                forward, left = self.yolo_track_velocity()
                self.publish_body_velocity(forward, left,
                                           self.clamp(self.cruise_height - p.z, 0.3))
                rospy.loginfo_throttle(1.0, "YOLO tracking error x=%.2f y=%.2f",
                                       self.yolo_error[0], self.yolo_error[1])
            else:
                self.publish_enu_velocity(0.0, 0.0, self.clamp(self.cruise_height - p.z, 0.3))
            if self.vehicle_visible() and tag is not None:
                rospy.loginfo("YOLO vehicle confirmed; AprilTag acquired")
                self.update_tag_world(tag)
                self.set_phase("TAG_APPROACH")
            else:
                rospy.logwarn_throttle(2.0, "waiting for YOLO vehicle + AprilTag")

        elif self.phase in ("TAG_APPROACH", "TAG_DEAD_RECKON"):
            if self.finish_platform_touchdown():
                return
            tag = self.tag_transform()
            if tag is not None and self.update_tag_world(tag):
                if self.phase == "TAG_DEAD_RECKON":
                    self.set_phase("TAG_APPROACH")
            elif self.phase == "TAG_APPROACH":
                self.set_phase("TAG_DEAD_RECKON")

            if self.person_pause and self.person_visible():
                self.publish_enu_velocity(0.0, 0.0, 0.0)
                rospy.logwarn_throttle(1.0, "person detected: descent paused")
                return

            clearance = (self.iris_world.z - self.platform_height
                         if self.iris_world is not None else
                         max(0.0, self.pose.pose.position.z - self.platform_height))
            vx, vy, horizontal = self.tag_guidance_velocity(clearance)
            down = self.descent_speed if clearance > 0.05 else 0.0

            # A frozen target is only trustworthy for the short blind terminal
            # segment. Never silently continue a long GPS-only approach.
            estimate_age = rospy.Time.now() - self.last_tag_update
            if (self.phase == "TAG_DEAD_RECKON" and
                    estimate_age > self.dead_reckon_timeout and clearance > 0.45):
                self.publish_enu_velocity(0.0, 0.0, 0.0)
                rospy.logerr_throttle(
                    1.0, "AprilTag estimate expired at %.2f m; holding", clearance)
                return

            self.publish_enu_velocity(vx, vy, -down)
            rospy.loginfo_throttle(
                0.5, "%s target=(%.2f, %.2f) error=%.2f clearance=%.2f "
                "velocity=(%.2f, %.2f, %.2f) age=%.2f",
                self.phase, self.tag_world[0], self.tag_world[1], horizontal,
                clearance, vx, vy, -down, estimate_age.to_sec())

        elif self.phase == "AUTO_LAND":
            if self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                try:
                    self.arm(False)
                except rospy.ServiceException:
                    pass
                self.set_phase("DONE")

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("vehicle_precision_landing")
    VehicleLanding().run()
