# -*- coding: utf-8 -*-
from __future__ import absolute_import

import logging
import os
import pkgutil
import types

import sys

from ._setup import _PyrosSetup, deprecated


delayed_import = _PyrosSetup.delayed_import
delayed_import_auto = _PyrosSetup.delayed_import_auto
delayed_import_new = _PyrosSetup.delayed_import_new


#
#  Until these are redefined by pyros_setup configuration, they are available with default configuration
#
def get_master(spawn=True):
    return delayed_import().get_master(spawn)


def get_ros_home():
    return delayed_import().get_ros_home()


__all__ = [
    'config',  # we expose the config subpackage if other want to use it (pyros does)

    'deprecated',

    'delayed_import',
    'delayed_import_auto',
    'delayed_import_new',

    'get_master',
    'get_ros_home',
]
