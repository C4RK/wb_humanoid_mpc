"""ROS remote-control package.

Keep optional joystick dependencies lazy so geometry and calibration utilities
can be imported on machines that do not have pygame installed.
"""

__all__ = ['XBoxControllerInterface']


def __getattr__(name):
    if name == 'XBoxControllerInterface':
        from .xbox_controller_interface import XBoxControllerInterface
        return XBoxControllerInterface
    raise AttributeError(name)
