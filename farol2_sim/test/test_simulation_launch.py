"""Clock/domain launch checks without DDS participants or sockets."""

from contextlib import contextmanager
import importlib.util
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import xacro
import yaml

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from rosgraph_msgs.msg import Clock
from farol2_sim import simulation_launch_utils as simulation


ROOT = Path(__file__).resolve().parents[2]


def setUpModule():
    global log_directory, log_environment
    log_directory = tempfile.TemporaryDirectory(prefix='farol2-launch-tests-')
    log_environment = patch.dict(os.environ, {'ROS_LOG_DIR': log_directory.name})
    log_environment.start()


def tearDownModule():
    log_environment.stop()
    log_directory.cleanup()


def launch_module(relative_path):
    spec = importlib.util.spec_from_file_location('tested_launch', ROOT / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ClockReadiness(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.node.count_publishers.return_value = 1
        self.wall_time = 0.0
        self.stamps = iter([])
        self.callback = None
        self.node.create_subscription.side_effect = self.subscribe
        self.executor = Mock()
        self.executor.spin_once.side_effect = lambda timeout_sec: self.spin(self.node, timeout_sec)

        @contextmanager
        def probe(domain):
            self.domain = domain
            yield self.node, self.executor

        for target, replacement in [
            ('_probe', probe), ('time.monotonic', lambda: self.wall_time),
        ]:
            context = patch('farol2_sim.simulation_launch_utils.' + target, replacement)
            context.start()
            self.addCleanup(context.stop)

    def subscribe(self, message_type, topic, callback, qos):
        self.assertIs(message_type, Clock)
        self.assertEqual(topic, '/clock')
        self.callback = callback
        self.qos = qos

    def spin(self, node, timeout_sec):
        self.wall_time += timeout_sec
        stamp = next(self.stamps, None)
        if stamp is not None:
            message = Clock()
            message.clock.sec, message.clock.nanosec = divmod(stamp, 1_000_000_000)
            self.callback(message)

    def test_advancing_clock_in_selected_domain(self):
        self.stamps = iter([0, 10_000_000])
        simulation.wait_for_clock('41', 0.2)
        self.assertEqual(self.domain, 41)
        self.assertEqual(self.qos.durability, simulation.DurabilityPolicy.VOLATILE)
        self.assertEqual(self.qos.reliability, simulation.ReliabilityPolicy.BEST_EFFORT)

    def test_missing_world_fails_with_domain_and_command(self):
        self.node.count_publishers.return_value = 0
        with self.assertRaisesRegex(RuntimeError, 'ROS_DOMAIN_ID=42.*Start the world first'):
            simulation.wait_for_clock(42, 0.2)

    def test_publisher_without_messages_is_not_ready(self):
        with self.assertRaisesRegex(RuntimeError, 'No advancing /clock'):
            simulation.wait_for_clock(41, 0.2)

    def test_one_retained_sample_is_not_ready(self):
        self.stamps = iter([100])
        with self.assertRaisesRegex(RuntimeError, 'No advancing /clock'):
            simulation.wait_for_clock(41, 0.2)

    def test_frozen_clock_is_not_ready(self):
        self.stamps = iter([100] * 10)
        with self.assertRaisesRegex(RuntimeError, 'No advancing /clock'):
            simulation.wait_for_clock(41, 0.2)

    def test_backward_jump_alone_is_not_ready(self):
        self.stamps = iter([100, 50])
        with self.assertRaisesRegex(RuntimeError, 'No advancing /clock'):
            simulation.wait_for_clock(41, 0.2)

    def test_clock_can_start_after_probe(self):
        self.stamps = iter([None, None, 0, 10])
        self.node.count_publishers.side_effect = [0, 0, 1, 1]
        simulation.wait_for_clock(41, 0.3)

    def test_duplicate_publishers_rejected(self):
        self.node.count_publishers.return_value = 2
        with self.assertRaisesRegex(RuntimeError, '2 /clock publishers'):
            simulation.wait_for_clock(41, 0.2)

    def test_world_rejects_occupied_domain_even_when_clock_frozen(self):
        with self.assertRaisesRegex(RuntimeError, 'already has a /clock'):
            simulation.ensure_clock_unowned(41, 0.2)

    def test_world_accepts_empty_domain(self):
        self.node.count_publishers.return_value = 0
        simulation.ensure_clock_unowned(42, 0.2)
        self.assertEqual(self.domain, 42)

    def test_invalid_timeout(self):
        for timeout in [0, -1, float('nan'), float('inf')]:
            with self.subTest(timeout=timeout), self.assertRaisesRegex(RuntimeError, 'clock_timeout'):
                simulation.wait_for_clock(41, timeout)


class LaunchPolicy(unittest.TestCase):
    def context(self, domain='41'):
        context = LaunchContext()
        context.launch_configurations.update(
            ros_domain_id=domain, clock_timeout='0.2', vehicle_name='mred',
            vehicle_id='0', config_to_use='default', world_config='/tmp/custom-world.yaml')
        for action in simulation.configure_domain(context):
            action.execute(context)
        return context

    def test_domains_are_validated_before_ros_initialization(self):
        for value in ['-1', '233', 'abc', '1.5']:
            with self.subTest(value=value), self.assertRaisesRegex(RuntimeError, 'ros_domain_id'):
                self.context(value)

    def test_domain_is_applied_to_child_environment(self):
        first = self.context('41')
        self.assertEqual(first.environment['ROS_DOMAIN_ID'], '41')
        second = self.context('42')
        self.assertEqual(second.environment['ROS_DOMAIN_ID'], '42')

    def test_world_defaults_to_zero_and_checks_ownership(self):
        module = launch_module('farol2_bringup/launch/start_simple_world.launch.py')
        description = module.generate_launch_description()
        domain = next(action for action in description.entities
                      if isinstance(action, DeclareLaunchArgument) and action.name == 'ros_domain_id')
        self.assertEqual(''.join(item.perform(LaunchContext()) for item in domain.default_value), '0')
        with patch.object(module, 'ensure_clock_unowned') as check:
            actions = module._start_world(self.context())
        check.assert_called_once_with('41')
        self.assertEqual(len(actions), 1)

    def test_vehicle_stack_waits_and_uses_simulation_time(self):
        module = launch_module('farol2_bringup/launch/start_vehicle_sim.launch.py')
        with patch.object(module, 'wait_for_clock') as check:
            actions = module._launch(self.context())
        check.assert_called_once_with(41, '0.2')
        self.assertEqual(len(actions), 3)  # ID, simulator, stack; no clock.
        arguments = dict(actions[-1].launch_arguments)
        self.assertEqual(arguments['use_sim_time'], 'true')
        self.assertEqual(dict(actions[-2].launch_arguments)['ros_domain_id'], '41')
        world = dict(actions[-2].launch_arguments)['world_config']
        self.assertEqual(world.perform(self.context()), '/tmp/custom-world.yaml')

    def test_missing_clock_prevents_entire_vehicle_stack(self):
        module = launch_module('farol2_bringup/launch/start_vehicle_sim.launch.py')
        with patch.object(module, 'wait_for_clock', side_effect=RuntimeError('No world')):
            with self.assertRaisesRegex(RuntimeError, 'No world'):
                module._launch(self.context())

    def test_auto_vehicle_id_discovery_uses_world_domain(self):
        module = launch_module('farol2_bringup/launch/start_vehicle_sim.launch.py')
        context = self.context('42')
        context.launch_configurations['vehicle_id'] = ''
        with patch.object(module, 'wait_for_clock'), \
                patch.object(module, 'get_next_available_vehicle_id', return_value='2') as discover:
            actions = module._launch(context)
        discover.assert_called_once_with('mred', domain_id=42)
        self.assertEqual(dict(actions[-1].launch_arguments)['vehicle_id'], '2')

    def test_standalone_simulator_also_requires_clock(self):
        module = launch_module('farol2_sim/launch/farol2_sim.launch.py')
        with patch.object(module, 'wait_for_clock', side_effect=RuntimeError('No world')):
            with self.assertRaisesRegex(RuntimeError, 'No world'):
                module._start_simulator(self.context(), object())
        node = object()
        with patch.object(module, 'wait_for_clock'):
            self.assertEqual(module._start_simulator(self.context(), node), [node])

    def test_standalone_simulator_loads_world_after_vehicle_parameters(self):
        module = launch_module('farol2_sim/launch/farol2_sim.launch.py')
        with patch.object(module, 'Node', wraps=module.Node) as create_node:
            module.generate_launch_description()
        parameters = create_node.call_args.kwargs['parameters']
        self.assertEqual(parameters[-2].perform(self.context()), '/tmp/custom-world.yaml')
        self.assertEqual(parameters[-1], {'use_sim_time': True})
        remappings = dict(create_node.call_args.kwargs['remappings'])
        altimeter_topic = ''.join(part.perform(self.context()) for part in remappings['altimeter'])
        self.assertEqual(altimeter_topic, '/mred0/measurement/altimeter')
        for name in ('thruster_rpm', 'control_surface_angle'):
            topic = ''.join(part.perform(self.context()) for part in remappings[name])
            self.assertEqual(topic, '/mred0/measurement/' + name)


class ProbeLifecycle(unittest.TestCase):
    def test_probe_uses_private_domain_context_and_cleans_up(self):
        context = Mock()
        context.ok.return_value = True
        node = Mock()
        executor = Mock()
        with patch.object(simulation, 'Context', return_value=context), \
                patch.object(simulation.rclpy, 'init') as init, \
                patch.object(simulation.rclpy, 'create_node', return_value=node), \
                patch.object(simulation, 'SingleThreadedExecutor', return_value=executor) as create_executor:
            with self.assertRaisesRegex(RuntimeError, 'test error'):
                with simulation._probe(42):
                    raise RuntimeError('test error')
        init.assert_called_once_with(args=[], context=context, domain_id=42)
        create_executor.assert_called_once_with(context=context)
        executor.add_node.assert_called_once_with(node)
        executor.shutdown.assert_called_once()
        node.destroy_node.assert_called_once()
        context.shutdown.assert_called_once()

    def test_clock_probe_never_uses_global_executor(self):
        context = Mock()
        node = Mock()
        node.count_publishers.return_value = 0
        executor = Mock()
        wall_time = [0.0]

        def spin(timeout_sec):
            wall_time[0] += timeout_sec

        executor.spin_once.side_effect = spin
        # Exercise ownership checking through the real _probe implementation,
        # rather than replacing _probe as the readiness state-machine tests do.
        with patch.object(simulation, 'Context', return_value=context), \
                patch.object(simulation.rclpy, 'init'), \
                patch.object(simulation.rclpy, 'create_node', return_value=node), \
                patch.object(simulation, 'SingleThreadedExecutor', return_value=executor) as create_executor, \
                patch.object(simulation.rclpy, 'get_global_executor', side_effect=AssertionError('Global context used')), \
                patch.object(simulation.time, 'monotonic', side_effect=lambda: wall_time[0]):
            simulation.ensure_clock_unowned(41, 0.1)
        create_executor.assert_called_once_with(context=context)
        self.assertGreater(executor.spin_once.call_count, 0)
        executor.shutdown.assert_called_once()


class MagicElectricActuators(unittest.TestCase):
    def setUp(self):
        config = yaml.safe_load((ROOT / 'farol2_bringup/config/magicelectric/default/sim.yaml').read_text())
        self.actuators = config['/**']['vehicle_sim']['ros__parameters']['vehicle']['actuators']

    def test_two_command_channels_without_fixed_rpm_or_double_thrust(self):
        self.assertEqual(len(self.actuators['frames']), 2)
        self.assertEqual(len(set(self.actuators['frames'])), 2)
        self.assertFalse(self.actuators['fixed_rpm'])
        self.assertEqual(self.actuators['thrust_scale'], 1.0)

    def test_allocation_and_simulator_use_the_same_physical_thrusters(self):
        path = str(ROOT / 'farol2_description/urdf/magicelectric.xacro')
        robot = xacro.process_file(path)
        joints = {joint.getElementsByTagName('child')[0].getAttribute('link'): joint
                  for joint in robot.getElementsByTagName('joint')}
        allocation = yaml.safe_load((ROOT / 'farol2_bringup/config/magicelectric/default/allocation.yaml').read_text())
        params = allocation['/**']['actuator_allocation']['ros__parameters']['allocation']
        self.assertEqual(params['type'], 'thrust_rudder')
        self.assertEqual(params['thrusters']['frames'], self.actuators['frames'])
        self.assertEqual(params['thrusters']['base_frame'], self.actuators['base_frame'])
        positions = []
        for frame in self.actuators['frames']:
            joint = joints[frame]
            self.assertEqual(joint.getAttribute('type'), 'fixed')
            self.assertEqual(joint.getElementsByTagName('parent')[0].getAttribute('link'),
                             self.actuators['base_frame'])
            origin = joint.getElementsByTagName('origin')[0]
            self.assertEqual([float(value) for value in origin.getAttribute('rpy').split()],
                             [0.0, 0.0, 0.0])
            positions.append([float(value) for value in origin.getAttribute('xyz').split()])
        # Actual TF lever arms must let equal forward forces produce pure surge.
        yaw_arms = [-position[1] for position in positions]
        self.assertAlmostEqual(sum(yaw_arms), 0.0)
        self.assertGreater(yaw_arms[0], 0.0)
        self.assertLess(yaw_arms[1], 0.0)
        self.assertAlmostEqual(sum(position[2] for position in positions), 0.0)
        dynamic_joints = [joint for joint in joints.values() if joint.getAttribute('type') != 'fixed']
        self.assertEqual(len(dynamic_joints), 1)
        config = yaml.safe_load((ROOT / 'farol2_bringup/config/magicelectric/default/sim.yaml').read_text())
        rudder = config['/**']['vehicle_sim']['ros__parameters']['vehicle']['rudder']
        self.assertEqual(dynamic_joints[0].getAttribute('name'), rudder['joint'])

class SensorMounts(unittest.TestCase):
    def test_enabled_sensor_frames_are_fixed_to_the_vehicle(self):
        for vehicle in ('myellow', 'mblack', 'mred', 'bluerov', 'delfim', 'magicelectric'):
            with self.subTest(vehicle=vehicle):
                config = yaml.safe_load((ROOT / 'farol2_bringup/config' / vehicle / 'default/sim.yaml').read_text())
                params = config['/**']['vehicle_sim']['ros__parameters']
                robot = xacro.process_file(str(ROOT / 'farol2_description/urdf' / (vehicle + '.xacro')))
                joints = {joint.getElementsByTagName('child')[0].getAttribute('link'): joint
                          for joint in robot.getElementsByTagName('joint')}
                for name in params['sensor_list']:
                    if name in ('thruster_rpm', 'control_surface_angle'):
                        continue
                    frame = params['sensor'][name]['frame']
                    visited = set()
                    while frame != params['vehicle']['actuators']['base_frame']:
                        self.assertNotIn(frame, visited, 'Cycle in sensor mounting chain')
                        visited.add(frame)
                        self.assertIn(frame, joints, 'Missing sensor mounting frame: ' + frame)
                        joint = joints[frame]
                        self.assertEqual(joint.getAttribute('type'), 'fixed')
                        origin = joint.getElementsByTagName('origin')[0]
                        for key in ('xyz', 'rpy'):
                            values = [float(value) for value in origin.getAttribute(key).split()]
                            self.assertEqual(len(values), 3)
                            self.assertTrue(all(math.isfinite(value) for value in values))
                        frame = joint.getElementsByTagName('parent')[0].getAttribute('link')


if __name__ == '__main__':
    unittest.main()
