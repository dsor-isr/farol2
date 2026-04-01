#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
import numpy as np

import math

def wrapTo2Pi(theta):
    return theta % (2 * math.pi)



class YawRefPublisher(Node):
    def __init__(self):
        super().__init__('yaw_ref_publisher')

        # Main ref
        self.pub_ref = self.create_publisher(
            Float32, '/magicelectric0/control/ref/yaw', 1
        )

        # Debug: (yaw, yaw_dot, yaw_ddot)
        self.pub_dbg = self.create_publisher(
            Vector3, '/magicelectric0/control/ref/yaw_dbg', 1
        )

        # Parameters
        self.declare_parameter('mode', 'sine')        # 'sine' or 'ramp'
        self.declare_parameter('amplitude', 1.0) # rad (sine only)
        self.declare_parameter('frequency', 0.01)     # Hz
        self.declare_parameter('offset', 0.0)         # rad
        self.declare_parameter('rate', 10.0)          # Hz

        self.mode = str(self.get_parameter('mode').value).lower()
        self.A = float(self.get_parameter('amplitude').value)
        self.f = float(self.get_parameter('frequency').value)
        self.offset = float(self.get_parameter('offset').value)
        rate = float(self.get_parameter('rate').value)

        if rate <= 0.0:
            raise ValueError("rate must be > 0")
        if self.f < 0.0:
            raise ValueError("frequency must be >= 0")

        self.dt = 1.0 / rate
        self.t = 0.0

        # Ramp state
        self.theta = 0.0
        self.omega = 0.1 #2.0 * math.pi * self.f  # rad/s

        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f"Yaw ref publisher started | mode={self.mode}, A={self.A}, "
            f"f={self.f} Hz, offset={self.offset}, rate={rate} Hz"
        )

    def timer_cb(self):
        yaw, yaw_dot, yaw_ddot = self.compute_ref(self.t, self.dt)

        # Publish main ref
        msg_ref = Float32()
        msg_ref.data = float(yaw)
        self.pub_ref.publish(msg_ref)

        # Publish debug triple
        msg_dbg = Vector3()
        msg_dbg.x = float(wrapTo2Pi(yaw))
        msg_dbg.y = float(yaw_dot)
        msg_dbg.z = float(yaw_ddot)
        self.pub_dbg.publish(msg_dbg)

        self.t += self.dt

    def compute_ref(self, t: float, dt: float):
        """
        Returns (yaw, yaw_dot, yaw_ddot) analytically.
        yaw is wrapped only in ramp mode (0..2π), like before.
        """
        if self.mode == 'sine':
            # yaw = offset + A sin(2π f t)
            w = 0.1 #2.0 * math.pi * self.f
            s = math.sin(w * t)
            c = math.cos(w * t)

            yaw = (self.offset + self.A * s) + np.random.uniform(-0.0001, 0.0001)
            yaw_dot = self.A * w * c
            yaw_ddot = -self.A * (w * w) * s
            return yaw, yaw_dot, yaw_ddot

        if self.mode == 'ramp':
            # theta evolves with constant angular speed omega
            # theta wrapped to [0, 2π)
            self.theta = (self.theta + self.omega * dt) % (2.0 * math.pi)

            yaw = self.offset + self.theta
            yaw_dot = self.omega
            yaw_ddot = 0.0
            return yaw, yaw_dot, yaw_ddot

        self.get_logger().warn(f"Unknown mode '{self.mode}'. Use 'sine' or 'ramp'.")
        return 0.0, 0.0, 0.0


def main():
    rclpy.init()
    node = YawRefPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
