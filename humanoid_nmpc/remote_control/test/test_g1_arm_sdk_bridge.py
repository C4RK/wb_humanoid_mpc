import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest


def _install_unitree_stubs_if_needed():
    try:
        import unitree_sdk2py  # noqa: F401
        return
    except ImportError:
        pass

    module_names = (
        'unitree_sdk2py',
        'unitree_sdk2py.core',
        'unitree_sdk2py.core.channel',
        'unitree_sdk2py.idl',
        'unitree_sdk2py.idl.default',
        'unitree_sdk2py.idl.unitree_hg',
        'unitree_sdk2py.idl.unitree_hg.msg',
        'unitree_sdk2py.idl.unitree_hg.msg.dds_',
        'unitree_sdk2py.utils',
        'unitree_sdk2py.utils.crc',
    )
    for name in module_names:
        sys.modules[name] = types.ModuleType(name)

    channel = sys.modules['unitree_sdk2py.core.channel']
    channel.ChannelFactoryInitialize = lambda *_args, **_kwargs: None
    channel.ChannelPublisher = type('ChannelPublisher', (), {})
    channel.ChannelSubscriber = type('ChannelSubscriber', (), {})

    default = sys.modules['unitree_sdk2py.idl.default']
    default.unitree_hg_msg_dds__LowCmd_ = lambda: object()

    dds = sys.modules['unitree_sdk2py.idl.unitree_hg.msg.dds_']
    dds.LowCmd_ = type('LowCmd_', (), {})
    dds.LowState_ = type('LowState_', (), {})

    crc = sys.modules['unitree_sdk2py.utils.crc']
    crc.CRC = type('CRC', (), {})


_install_unitree_stubs_if_needed()
SCRIPT_PATH = Path(__file__).parents[1] / 'scripts' / 'g1_arm_sdk_bridge.py'
SPEC = importlib.util.spec_from_file_location('g1_arm_sdk_bridge', SCRIPT_PATH)
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


class G1ArmSdkBridgeTest(unittest.TestCase):

    def test_stable_target_uses_component_medians(self):
        centre = [0.1, 0.2, -0.3, 0.4, -0.5]
        samples = [
            tuple(value + math.radians(offset) for value in centre)
            for offset in (-0.5, -0.2, 0.0, 0.2, 0.5) * 2
        ]

        result = BRIDGE.stable_target_from_samples(
            samples,
            frozenset(BRIDGE.LEFT_ARM_NAMES),
            math.radians(1.0),
        )

        for actual, expected in zip(result, centre):
            self.assertAlmostEqual(actual, expected)

    def test_stable_target_rejects_motion_on_an_active_joint_only(self):
        quiet = (0.0, 0.0, 0.0, 0.0, 0.0)
        moving = (0.0, math.radians(6.0), 0.0, 0.0, 0.0)
        samples = [quiet] * 9 + [moving]

        with self.assertRaisesRegex(
                RuntimeError, 'human roll target was not stable'):
            BRIDGE.stable_target_from_samples(
                samples, frozenset({'roll'}), math.radians(2.0))

        result = BRIDGE.stable_target_from_samples(
            samples, frozenset({'elbow'}), math.radians(2.0))
        for actual, expected in zip(result, quiet):
            self.assertAlmostEqual(actual, expected)

    def test_absolute_mapping_uses_source_target_for_active_joints(self):
        initial = {index: 0.75 for index in BRIDGE.CONTROLLED_JOINTS}
        source = (0.1, 0.2, 0.3, 0.4, 0.5)

        mapped = BRIDGE.map_target_to_robot(
            source, source, initial, False, frozenset({'elbow'}))

        expected = (0.75, 0.75, 0.75, 0.4, 0.75)
        for actual, wanted in zip(mapped, expected):
            self.assertAlmostEqual(actual, wanted)

    def test_rate_limited_step_moves_each_joint_toward_target(self):
        desired = {index: 0.0 for index in BRIDGE.CONTROLLED_JOINTS}
        target = (0.5, -0.5, 0.01, -0.01, 0.0)

        BRIDGE.step_desired_arm_toward(desired, target, max_step=0.02)

        actual = tuple(
            desired[index] for index, _name, _limits in BRIDGE.LEFT_ARM)
        expected = (0.02, -0.02, 0.01, -0.01, 0.0)
        for value, wanted in zip(actual, expected):
            self.assertAlmostEqual(value, wanted)

    def test_active_difference_ignores_inactive_joints(self):
        first = (0.0, 1.0, 2.0, 0.3, 4.0)
        second = (0.0, 0.0, 0.0, 0.1, 0.0)

        difference, name = BRIDGE.maximum_active_target_difference(
            first, second, frozenset({'elbow'}))

        self.assertEqual(name, 'elbow')
        self.assertAlmostEqual(difference, 0.2)

    def test_safe_range_clips_absolute_target_about_initial_pose(self):
        initial = {index: 0.0 for index in BRIDGE.CONTROLLED_JOINTS}
        elbow_index = next(
            index for index, name, _limits in BRIDGE.LEFT_ARM
            if name == 'elbow')
        initial[elbow_index] = math.radians(55.0)
        limits = {name: math.radians(20.0) for name in BRIDGE.LEFT_ARM_NAMES}
        limits['elbow'] = math.radians(15.0)

        bounded = BRIDGE.bound_target_to_safe_range(
            (0.0, 0.0, 0.0, 0.0, 0.0), initial, limits)

        self.assertAlmostEqual(math.degrees(bounded[3]), 40.0)


if __name__ == '__main__':
    unittest.main()
