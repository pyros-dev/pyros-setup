# -*- coding: utf-8 -*-
from __future__ import absolute_import

from ._version import __version__

import logging
import os
import pkgutil
import types

import sys

from .utils import deprecated
from .config import ConfigImport, ConfigHandler


# class to allow delayed conditional import, with behavior based on configuration.
# This way it can work with or without preset environment
class _PyrosSetup(ConfigImport):
    #: Default configuration parameters.
    _default_config = {
        'DISTRO': 'indigo',
        'WORKSPACES': [],
    }

    def __init__(self, instance_path=None, instance_relative_config=True, root_path=None):

        relay_import_dict = {
            'rospy': 'rospy',  # early except to detect errors and prevent unintentional useless workarounds
            'ros_utils': ('.ros_utils', __package__),
            'rostest_nose': ('.rostest_nose', __package__),
        }
        fix_imports = self._attempt_import_fix

        super(_PyrosSetup, self).__init__('pyros_setup',
                                          'Small setup package to dynamically interface with ROS',
                                          relay_import_dict=relay_import_dict,
                                          fix_imports=fix_imports,
                                          instance_path=instance_path,
                                          instance_relative_config=instance_relative_config,
                                          root_path=root_path,
                                          default_config=self._default_config)

    def _attempt_import_fix(self):
        from .ros_setup import ROS_emulate_setup
        # we want this to expect in case of bad config, because default_config has to have these fields.
        ROS_emulate_setup(self.config['DISTRO'], *self.config['WORKSPACES'])

    def activate(self):
        super(_PyrosSetup, self).activate()

        # defining shortcuts after successful import
        self.get_master = self.ros_utils.get_master
        self.get_ros_home = self.ros_utils.get_ros_home

        return self


    # encapsulating local imports to delay them until ROS setup is done
    @classmethod
    @deprecated
    def delayed_import(cls, distro=None, *workspaces):

        # building a one time config for bwcompat purpose
        cfg = cls._default_config
        cfg['DISTRO'] = distro or cls._default_config['DISTRO']
        cfg['WORKSPACES'] = workspaces
        module_redirect = cls()
        module_redirect.configure(cfg)
        module_redirect.activate()

        return module_redirect

    @classmethod
    @deprecated
    def delayed_import_auto(cls, distro=None, base_path=None):
        import os

        distro = distro or cls._default_config['DISTRO']
        # default basepath working if pyros-setup is directly cloned in your workspace
        # This file think it is in devel/lib/python2.7/dist-packages/pyros_setup for some reason...
        # NOTE : maybe the default here is a bad idea, making it simpler than it should be for the user...
        base_path = base_path or os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')

        # using this for bwcompat purpose only. now seems like a bad idea after all.
        from .ros_setup import ROS_find_workspaces
        workspaces = ROS_find_workspaces(distro, base_path)

        return cls.delayed_import()

    #
    # Factory method
    #

    @classmethod
    def configurable_import(cls, instance_path=None, instance_relative_config=True, root_path=None):
        # we return a relay of imported names, accessible the same way a direct import would be.
        module_redirect = cls(
            instance_path=instance_path,
            instance_relative_config=instance_relative_config,
            root_path=root_path
        )
        return module_redirect

delayed_import = _PyrosSetup.delayed_import
delayed_import_auto = _PyrosSetup.delayed_import_auto
configurable_import = _PyrosSetup.configurable_import

#
#  Until these are redefined by pyros_setup configuration, they are available with default configuration
#
def get_master(spawn=True):
    return delayed_import().get_master(spawn)


def get_ros_home():
    return delayed_import().get_ros_home()


__all__ = [
    '__version__',
    'ConfigHandler',  # we expose the config subpackage if other want to use it (pyros does)

    'deprecated',

    'delayed_import',
    'delayed_import_auto',
    'configurable_import',

    'get_master',
    'get_ros_home',
]
