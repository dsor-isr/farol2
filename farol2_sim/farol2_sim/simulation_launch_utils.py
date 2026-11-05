"""Shared launch utilities for world and vehicle simulation launchers.

Validate the requested DDS domain and set ROS_DOMAIN_ID for launched processes.
Before starting a vehicle, require exactly one publisher of fresh, advancing
/clock messages. Before starting a world, reject a domain that already has a
/clock publisher, even if that clock is frozen.

Checks use a temporary ROS node with its own context and executor in the selected
domain, and clean those resources up afterward. Discovery and readiness deadlines
use monotonic wall time so a missing or paused simulation clock cannot stall launch
indefinitely. This module neither publishes /clock nor runs vehicle physics.
"""

from contextlib import contextmanager
import math
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def domain_id(value):
    """Validate the DDS domain range supported by this Linux simulation."""
    try:
        domain = int(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError('ros_domain_id must be an integer in 0..232') from error
    if not 0 <= domain <= 232:
        raise RuntimeError('ros_domain_id must be an integer in 0..232')
    return domain


def configure_domain(context):
    # Set the environment inherited by nodes in this launch process. Probes
    # explicitly initialize a private rclpy context in the selected domain.
    from launch.actions import SetEnvironmentVariable
    from launch.substitutions import LaunchConfiguration

    domain = domain_id(LaunchConfiguration('ros_domain_id').perform(context))
    return [SetEnvironmentVariable('ROS_DOMAIN_ID', str(domain))]


@contextmanager
def _probe(domain):
    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(args=[], context=context, domain_id=domain)
        node = rclpy.create_node(
            '_simulation_clock_probe', context=context,
            use_global_arguments=False, enable_rosout=False,
            start_parameter_services=False)
        # rclpy.spin_once(node) otherwise creates the global executor using the
        # uninitialized default context, rather than this node's private context.
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        yield node, executor
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def wait_for_clock(domain, timeout=5.0):
    """Require one publisher and fresh, advancing /clock messages.

    All deadlines use monotonic wall time, so an absent or frozen simulation
    clock cannot deadlock launch. Volatile/best-effort subscription accepts both
    reliable and best-effort sources without accepting only a retained sample.
    """
    domain = domain_id(domain)
    timeout = float(timeout)
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise RuntimeError('clock_timeout must be finite and positive (wall seconds)')
    first_stamp = None
    advancing = False

    def receive(message):
        nonlocal first_stamp, advancing
        stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if first_stamp is None or stamp < first_stamp:
            first_stamp = stamp
        elif stamp > first_stamp:
            advancing = True

    with _probe(domain) as (node, executor):
        node.create_subscription(Clock, '/clock', receive, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            count = node.count_publishers('/clock')
            if count > 1:
                raise RuntimeError(
                    f'ROS_DOMAIN_ID={domain} has {count} /clock publishers. '
                    'Use exactly one world per domain.')
            if advancing and count == 1:
                return
    raise RuntimeError(
        f'No advancing /clock received within {timeout:g} wall seconds in '
        f'ROS_DOMAIN_ID={domain}. Start the world first: '
        f'ros2 launch farol2_bringup start_simple_world.launch.py ros_domain_id:={domain}')


def ensure_clock_unowned(domain, discovery_time=0.5):
    """Reject an already occupied world domain, including a frozen clock."""
    domain = domain_id(domain)
    with _probe(domain) as (node, executor):
        deadline = time.monotonic() + discovery_time
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
            if node.count_publishers('/clock'):
                raise RuntimeError(
                    f'ROS_DOMAIN_ID={domain} already has a /clock publisher. '
                    'Join that world or choose a different ros_domain_id.')
