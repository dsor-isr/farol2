"""Exercise all shipped models with real ROS nodes, Xacro TF and one test clock.

Run in an isolated ROS domain after sourcing ROS and farol2_interfaces:
  ROS_DOMAIN_ID=159 python3 check_vehicle_configs.py /path/to/vehicle_sim /path/to/farol2
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from farol2_interfaces.msg import ControlSurfaceAngle, ThrusterRPM
import xacro
import yaml


def main():
    executable, root = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    rsp = Path(get_package_prefix('robot_state_publisher')) / 'lib/robot_state_publisher/robot_state_publisher'
    rclpy.init()
    node = rclpy.create_node('vehicle_model_check')
    clock = node.create_publisher(Clock, '/clock', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE))
    samples, publishers, subscriptions, processes = {}, {}, [], []
    with tempfile.TemporaryDirectory(prefix='farol2-model-check-') as tmp:
        logs = []

        def start(command, name):
            log = open(Path(tmp) / f'{name}.log', 'w+')
            logs.append((name, log))
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            processes.append(process)
            return process

        def callback(key, message):
            values = [message.x, message.y, message.z]
            assert all(math.isfinite(v) for v in values), (key, values)
            samples.setdefault(key, []).append(values)

        try:
            for path in sorted((root / 'farol2_bringup/config').glob('*/default/sim.yaml')):
                name = path.parent.parent.name
                ns = f'/model_check/{name}'
                prefix = f'model_check/{name}/'
                parameters = yaml.safe_load(path.read_text())['/**']['vehicle_sim']['ros__parameters']
                frames = parameters['vehicle']['actuators']['frames']
                description = xacro.process_file(str(root / f'farol2_description/urdf/{name}.xacro')).toxml()
                start([str(rsp), '--ros-args', '-r', f'__ns:={ns}',
                       '-p', f'robot_description:={description}', '-p', f'frame_prefix:={prefix}',
                       '-p', 'use_sim_time:=true'], f'{name}_tf')
                start([str(executable), '--ros-args', '--params-file', str(path),
                       '--params-file', str(root / 'farol2_bringup/config/simple_world.yaml'),
                       '-r', f'__ns:={ns}', '-p', f'frame_prefix:={prefix}',
                       '-p', 'use_sim_time:=true',
                       '-r', f'joint_states:={ns}/joint_states'], name)
                publishers[name] = (
                    node.create_publisher(ThrusterRPM, f'{ns}/rpm_command', 1),
                    node.create_publisher(ControlSurfaceAngle, f'{ns}/control_surface_angle_ref', 1),
                    len(frames))
                for topic in ('position', 'body_velocity', 'orientation'):
                    key = (name, topic)
                    subscriptions.append(node.create_subscription(Vector3, f'{ns}/{topic}',
                        lambda msg, k=key: callback(k, msg), 10))
                if name == 'magicelectric':
                    subscriptions.append(node.create_subscription(JointState, f'{ns}/joint_states',
                        lambda msg: samples.setdefault(('magicelectric', 'rudder'), []).append(msg.position), 10))

            # Step at approximately real time to allow discovery and ROS timers.
            for tick in range(800):
                message = Clock()
                message.clock.sec, message.clock.nanosec = divmod((tick + 1) * 10_000_000, 1_000_000_000)
                clock.publish(message)
                for rpm_pub, rudder_pub, count in publishers.values():
                    rpm = ThrusterRPM(); rpm.rpm = [1000.0] * count
                    rpm_pub.publish(rpm)
                    rudder = ControlSurfaceAngle(); rudder.angle = [10.0]
                    rudder_pub.publish(rudder)
                deadline = time.monotonic() + 0.01
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.001)
                assert all(p.poll() is None for p in processes), 'A vehicle or TF publisher exited'

            for name in publishers:
                positions = samples.get((name, 'position'), [])
                assert len(positions) > 10, (name, 'No simulation output')
                assert abs(positions[-1][0]) > 0.001, (name, 'No commanded surge motion')
                if name == 'magicelectric':
                    assert all(abs(p[2]) < 1e-12 for p in positions), 'Planar depth changed'
                    orientations = samples[(name, 'orientation')]
                    assert all(abs(o[0]) < 1e-12 and abs(o[1]) < 1e-12 for o in orientations)
                    assert abs(orientations[-1][2]) > 0.01, 'Rudder produced no yaw'
                    assert abs(samples[(name, 'body_velocity')][-1][1]) > 1e-5, 'No sway response'
                    assert abs(samples[(name, 'rudder')][-1][0] - math.radians(10.0)) < 1e-8
                print(f'{name}: {len(positions)} finite states; final position {positions[-1]}')

            # Configuration validation must fail before starting simulation.
            bad = start([str(executable), '--ros-args', '--params-file',
                str(root / 'farol2_bringup/config/mred/default/sim.yaml'),
                '-p', 'vehicle.mass:=-1.0'], 'invalid_config')
            assert bad.wait(timeout=10) == 1, 'Invalid mass was accepted'
            print('Invalid configuration rejected; all vehicle checks passed')
        except Exception:
            for name, log in logs:
                log.flush(); log.seek(0)
                print(f'--- {name} ---\n{log.read()}', file=sys.stderr)
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
            for _, log in logs:
                log.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    # The caller chooses a dedicated domain to avoid other running vehicles.
    if 'ROS_DOMAIN_ID' not in os.environ:
        raise SystemExit('Set an isolated ROS_DOMAIN_ID before running this check')
    main()
