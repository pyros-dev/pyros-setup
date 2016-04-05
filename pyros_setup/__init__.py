# -*- coding: utf-8 -*-
from __future__ import absolute_import

import logging
import os
import pkgutil
import types

import sys

from ._setup import _PyrosSetup


delayed_import = _PyrosSetup.delayed_import
delayed_import_auto = _PyrosSetup.delayed_import_auto
delayed_import_config = _PyrosSetup.delayed_import_config


#
#  Until these are redefined by pyros_setup configuration, they are available with default configuration
#
def get_master(spawn=True):
    return delayed_import().get_master(spawn)


def get_ros_home():
    return delayed_import().get_ros_home()


__all__ = [
    'delayed_import',
    'delayed_import_auto',
    'delayed_import_config',

    'get_master',
    'get_ros_home',
]
