# -*- coding: utf-8 -*-
from __future__ import absolute_import


# class to allow (potentially infinite) delayed conditional import.
# this way it can work with or without preset environment
class _PyrosSetup(object):
    def __init__(self, ros_master):
        self.get_master = ros_master

    # encapsulating local imports to delay them until ROS setup is done
    @staticmethod
    def delayed_import():
        try:
            import rospy  # early except to prevent unintentional workaround in all modules here
            from .ros_utils import get_master
        except ImportError:
            from .ros_setup import ROS_emulate_setup
            ROS_emulate_setup()
            import rospy
            from .ros_utils import get_master

        # we return a relay of imported names, accessible the same way a direct import would be.
        return _PyrosSetup(get_master)

delayed_import = _PyrosSetup.delayed_import

__all__ = [
    'delayed_import',
]