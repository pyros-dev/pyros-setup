# -*- coding: utf-8 -*-
from __future__ import absolute_import

import types

# class to allow (potentially infinite) delayed conditional import.
# this way it can work with or without preset environment
class _PyrosSetup(types.ModuleType):
    def __init__(self, ros_master):
        super(_PyrosSetup, self).__init__('pyros_setup','Small setup package to dynamically interface with ROS')
        # members simulating a usual imported module
        self.get_master = ros_master

    # encapsulating local imports to delay them until ROS setup is done
    @staticmethod
    def delayed_import(distro=None, *workspaces):
        distro = distro or 'indigo'
        try:
            import rospy  # early except to prevent unintentional workaround in all modules here
            from .ros_utils import get_master
        except ImportError:
            from .ros_setup import ROS_emulate_setup
            ROS_emulate_setup(distro, *workspaces)
            import rospy
            from .ros_utils import get_master

        # we return a relay of imported names, accessible the same way a direct import would be.
        return _PyrosSetup(get_master)

    @staticmethod
    def delayed_import_auto(distro=None, base_path=None):
        import os

        distro = distro or 'indigo'
        # default basepath working if pyros-setup is directly cloned in your workspace
        # This file think it is in devel/lib/python2.7/dist-packages/pyros_setup for some reason...
        # NOTE : maybe the default here is a bad idea, making it simpler than it should be for the user...
        base_path = base_path or os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')

        try:
            import rospy  # early except to prevent unintentional workaround in all modules here
            from .ros_utils import get_master
        except ImportError:
            from .ros_setup import ROS_find_workspaces, ROS_emulate_setup
            workspaces = ROS_find_workspaces(distro, base_path)
            ROS_emulate_setup(distro, *workspaces)
            import rospy
            from .ros_utils import get_master

        # we return a relay of imported names, accessible the same way a direct import would be.
        return _PyrosSetup(get_master)

delayed_import = _PyrosSetup.delayed_import
delayed_import_auto = _PyrosSetup.delayed_import_auto


def get_master(spawn=True):
    return delayed_import().get_master(spawn)


__all__ = [
    'delayed_import',
    'delayed_import_auto',
    'get_master',
]
