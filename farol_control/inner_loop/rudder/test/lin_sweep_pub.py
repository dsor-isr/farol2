#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ============================
# EDIT PARAMETERS HERE (RAD/S)
# Option A: 0.05 -> 0.6 rad/s in ~10.4 min
# ============================

TOPIC = "/magicelectric0/control/ref/rudder_angle"
RATE_HZ = 10.0      # publish rate (Hz)

A = 0.25              # amplitude
OFFSET = 0.0         # DC offset

W0 = 0.2            # start frequency (rad/s)
W1 = 1.0           # end frequency (rad/s)
ALPHA = 1.0e-3       # sweep rate (rad/s^2): w(t) = W0 + ALPHA*t

HOLD_END_S = 0.0     # optional hold at W1 after sweep (seconds)

# ============================


class LinearSweepPub(Node):
    def __init__(self):
        super().__init__("linear_sweep_pub")
        self.pub = self.create_publisher(Float32, TOPIC, 10)

        if RATE_HZ <= 0:
            raise ValueError("RATE_HZ must be > 0")
        if ALPHA <= 0:
            raise ValueError("ALPHA must be > 0")
        if W1 < W0:
            raise ValueError("W1 must be >= W0")

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

        self.T_sweep = 0.0 if W1 == W0 else (W1 - W0) / ALPHA
        self.get_logger().info(
            f"Publishing linear chirp on {TOPIC} at {RATE_HZ:.1f} Hz | "
            f"w0={W0} rad/s -> w1={W1} rad/s at alpha={ALPHA} rad/s^2 | "
            f"T_sweep={self.T_sweep:.2f} s"
        )

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # Stop after sweep (+ optional hold)
        if t >= self.T_sweep + max(0.0, HOLD_END_S):
            rclpy.shutdown()
            return

        if t <= self.T_sweep:
            # Linear chirp in rad/s:
            # w(t) = W0 + ALPHA*t
            # phase(t) = ∫ w(t) dt = W0*t + 0.5*ALPHA*t^2
            phase = (W0 * t) + (0.5 * ALPHA * t * t)
        else:
            # Hold end frequency with continuous phase
            t1 = self.T_sweep
            phase_at_t1 = (W0 * t1) + (0.5 * ALPHA * t1 * t1)
            phase = phase_at_t1 + (W1 * (t - t1))

        x = OFFSET + A * math.sin(phase)

        msg = Float32()
        msg.data = float(x)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LinearSweepPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
