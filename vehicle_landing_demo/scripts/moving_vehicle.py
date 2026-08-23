#!/usr/bin/env python3
"""Drive the landing vehicle with physical force, never pose teleportation."""

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench


class PhysicalVehicleDriver(object):
    def __init__(self):
        self.speed = rospy.get_param("~speed", 0.20)
        self.start_delay = rospy.get_param("~start_delay", 8.0)
        self.target_y = rospy.get_param("~initial_y", 0.0)
        self.mass = rospy.get_param("~mass", 1200.0)
        self.longitudinal_kp = rospy.get_param("~longitudinal_kp", 5000.0)
        self.longitudinal_ki = rospy.get_param("~longitudinal_ki", 8000.0)
        self.drive_force_limit = rospy.get_param("~drive_force_limit", 18000.0)
        self.speed_integral_limit = rospy.get_param("~speed_integral_limit", 3.0)
        self.speed_integral = 0.0
        self.pose = None
        self.twist = None
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.models_cb, queue_size=1)
        self.apply_wrench = rospy.ServiceProxy(
            "/gazebo/apply_body_wrench", ApplyBodyWrench, persistent=True)

    def models_cb(self, msg):
        try:
            index = msg.name.index("ugv_0")
            self.pose = msg.pose[index]
            self.twist = msg.twist[index]
        except ValueError:
            pass

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def run(self):
        rospy.wait_for_service("/gazebo/apply_body_wrench")
        rospy.sleep(self.start_delay)
        rate = rospy.Rate(20)
        previous_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.pose is None or self.twist is None:
                rate.sleep()
                continue

            wrench = Wrench()
            # Longitudinal velocity servo. Force is applied by Gazebo's physics
            # engine, so contact friction can carry the disarmed aircraft.
            now = rospy.Time.now()
            dt = min(0.2, max(0.0, (now - previous_time).to_sec()))
            previous_time = now
            speed_error = self.speed - self.twist.linear.x
            if abs(self.speed) < 1e-3:
                self.speed_integral = 0.0
            else:
                self.speed_integral = self.clamp(
                    self.speed_integral + speed_error * dt,
                    self.speed_integral_limit)
            wrench.force.x = self.clamp(
                self.longitudinal_kp * speed_error
                + self.longitudinal_ki * self.speed_integral,
                self.drive_force_limit)
            # Keep the free rigid body centred on the straight road.
            wrench.force.y = self.clamp(
                -2200.0 * (self.pose.position.y - self.target_y)
                - 1400.0 * self.twist.linear.y, 700.0)
            wrench.torque.z = self.clamp(
                -1800.0 * self.twist.angular.z, 500.0)
            try:
                self.apply_wrench(
                    body_name="ugv_0::body", reference_frame="world",
                    reference_point=Point(), wrench=wrench,
                    start_time=rospy.Time(0), duration=rospy.Duration(0.08))
            except rospy.ServiceException as exc:
                rospy.logwarn_throttle(2.0, "vehicle wrench failed: %s", exc)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("moving_landing_vehicle")
    PhysicalVehicleDriver().run()
