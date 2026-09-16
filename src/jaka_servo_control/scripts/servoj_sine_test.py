#!/usr/bin/env python3
"""Publish a small, smooth one-joint ServoJ bench-test command."""

import math

import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, SetBoolRequest


class ServoJSineTest:
    def __init__(self):
        self._rate_hz = float(rospy.get_param("~control_rate", 125.0))
        self._joint_index = int(rospy.get_param("~joint_index", 5))
        self._amplitude = float(rospy.get_param("~amplitude", 0.05))
        self._frequency = float(rospy.get_param("~frequency", 0.15))
        self._max_joint_step = float(rospy.get_param("~max_joint_step", 0.001))

        if not 0 <= self._joint_index < 6:
            raise rospy.ROSInitException("joint_index must be in [0, 5]")
        if self._rate_hz <= 0.0 or self._amplitude <= 0.0 or self._frequency <= 0.0:
            raise rospy.ROSInitException("control_rate, amplitude and frequency must be positive")

        self._publisher = rospy.Publisher(
            "/jaka_servo_control/joint_increment", Float64MultiArray, queue_size=1
        )
        rospy.wait_for_service("/jaka_servo_control/enable")
        self._enable = rospy.ServiceProxy("/jaka_servo_control/enable", SetBool)

        response = self._enable(SetBoolRequest(data=True))
        if not response.success:
            raise rospy.ROSInitException("Cannot enable ServoJ bridge: %s" % response.message)

        self._start_time = rospy.Time.now().to_sec()
        self._previous_position = 0.0
        rospy.on_shutdown(self._shutdown)
        rospy.logwarn(
            "ServoJ sine test started: J%d, amplitude %.3f rad, frequency %.3f Hz",
            self._joint_index + 1,
            self._amplitude,
            self._frequency,
        )

    def run(self):
        rate = rospy.Rate(self._rate_hz)
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - self._start_time
            position = self._amplitude * math.sin(2.0 * math.pi * self._frequency * elapsed)
            increment = position - self._previous_position
            self._previous_position = position

            increment = max(-self._max_joint_step, min(self._max_joint_step, increment))
            command = [0.0] * 6
            command[self._joint_index] = increment
            self._publisher.publish(Float64MultiArray(data=command))
            rate.sleep()

    def _shutdown(self):
        try:
            self._publisher.publish(Float64MultiArray(data=[0.0] * 6))
            self._enable(SetBoolRequest(data=False))
        except rospy.ServiceException:
            pass


if __name__ == "__main__":
    rospy.init_node("servoj_sine_test")
    ServoJSineTest().run()
