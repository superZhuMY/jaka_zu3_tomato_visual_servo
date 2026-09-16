#!/usr/bin/env python3
"""Fixed-rate, guarded ROS1 ServoJ bridge for the existing JAKA service driver."""

import math
import threading

import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, SetBoolResponse
from jaka_msgs.srv import (
    ServoMove,
    ServoMoveRequest,
    ServoMoveEnable,
    ServoMoveEnableRequest,
)


class ServoJBridge:
    def __init__(self):
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._enabled = False
        self._latest_increment = None
        self._last_command_time = rospy.Time(0)

        self._rate_hz = float(rospy.get_param("~control_rate", 125.0))
        self._timeout = float(rospy.get_param("~command_timeout", 0.10))
        self._max_joint_step = float(rospy.get_param("~max_joint_step", 0.001))

        if self._rate_hz <= 0.0 or self._timeout <= 0.0 or self._max_joint_step <= 0.0:
            raise rospy.ROSInitException("control_rate, command_timeout and max_joint_step must be positive")

        rospy.wait_for_service("/jaka_driver/servo_move_enable")
        rospy.wait_for_service("/jaka_driver/servo_j")
        self._enable_client = rospy.ServiceProxy(
            "/jaka_driver/servo_move_enable", ServoMoveEnable
        )
        self._servoj_client = rospy.ServiceProxy("/jaka_driver/servo_j", ServoMove)

        self._command_sub = rospy.Subscriber(
            "joint_increment", Float64MultiArray, self._command_callback, queue_size=1
        )
        self._enable_service = rospy.Service("enable", SetBool, self._enable_callback)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._rate_hz), self._control_timer
        )
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "ServoJ bridge ready: %.1f Hz, timeout %.3f s, max step %.6f rad",
            self._rate_hz,
            self._timeout,
            self._max_joint_step,
        )

    def _command_callback(self, message):
        values = list(message.data)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, "Ignoring invalid ServoJ command; exactly 6 finite values are required")
            return

        clamped = [
            max(-self._max_joint_step, min(self._max_joint_step, float(value)))
            for value in values
        ]
        if clamped != values:
            rospy.logwarn_throttle(1.0, "ServoJ command was clamped to max_joint_step")

        with self._lock:
            self._latest_increment = clamped
            self._last_command_time = rospy.Time.now()

    def _enable_callback(self, request):
        if request.data:
            with self._lock:
                if self._enabled:
                    return SetBoolResponse(True, "ServoJ bridge is already enabled")
                self._latest_increment = None
                self._last_command_time = rospy.Time.now()

            try:
                response = self._enable_client(ServoMoveEnableRequest(enable=True))
            except rospy.ServiceException as error:
                return SetBoolResponse(False, "Failed to enable JAKA servo mode: %s" % error)

            if response.ret != 1:
                return SetBoolResponse(False, response.message)

            with self._lock:
                self._enabled = True
            return SetBoolResponse(True, "JAKA servo mode enabled")

        self._safe_disable("disable requested")
        return SetBoolResponse(True, "JAKA servo mode disabled")

    def _send_increment(self, increment):
        request = ServoMoveRequest()
        request.pose = increment
        request.speed = []
        response = self._servoj_client(request)
        return response.ret == 1, response.message

    def _control_timer(self, _event):
        with self._lock:
            enabled = self._enabled
            increment = self._latest_increment
            command_age = (rospy.Time.now() - self._last_command_time).to_sec()

        if not enabled:
            return

        if increment is None or command_age > self._timeout:
            self._safe_disable("ServoJ command timeout")
            return

        if not self._send_lock.acquire(False):
            return

        try:
            ok, message = self._send_increment(increment)
            if not ok:
                rospy.logerr("ServoJ command rejected by driver: %s", message)
                self._safe_disable("driver rejected ServoJ command")
        except rospy.ServiceException as error:
            rospy.logerr("ServoJ service call failed: %s", error)
            self._safe_disable("ServoJ service call failed")
        finally:
            self._send_lock.release()

    def _safe_disable(self, reason):
        with self._lock:
            if not self._enabled:
                return
            self._enabled = False
            self._latest_increment = None

        rospy.logwarn("Stopping ServoJ bridge: %s", reason)
        zero = [0.0] * 6
        try:
            for _ in range(2):
                self._send_increment(zero)
            self._enable_client(ServoMoveEnableRequest(enable=False))
        except rospy.ServiceException as error:
            rospy.logerr("Could not cleanly disable JAKA servo mode: %s", error)

    def _shutdown(self):
        self._safe_disable("ROS shutdown")


if __name__ == "__main__":
    rospy.init_node("jaka_servo_control")
    ServoJBridge()
    rospy.spin()
