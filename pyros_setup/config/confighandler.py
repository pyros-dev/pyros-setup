# -*- coding: utf-8 -*-
"""
    pyros_setup.config.confighandler
    ~~~~~~~~~~~~~

    Implements a class to access configuration files
    Inspired from Flask
"""

import os
import pkgutil
import sys
from threading import RLock

from .config import Config

from .config import Config
from .packagebound import PackageBound
from .helpers import locked_cached_property


class ConfigHandler(PackageBound):
    config_class = Config

    def __init__(self, import_name, instance_path=None, instance_relative_config=True, root_path=None, default_config=None):

        #: default config passed to the constructor. should not be changed
        self.default_config = default_config or {}

        super(ConfigHandler, self).__init__(import_name,
                                            instance_path=instance_path,
                                            instance_relative_config=instance_relative_config,
                                            root_path=root_path)

        #: The configuration dictionary as :class:`Config`.  This behaves
        #: exactly like a regular dictionary but supports additional methods
        #: to load a config from files.
        self.config = self.make_config(instance_relative_config)

    @locked_cached_property
    def instance_relative_config(self):
        return self.config.root_path == self.instance_path

    # This function attempts to find out what was the configuration passed
    # And retrieve values accordingly...
    def configure(self, config):
        if isinstance(config, (str, unicode)):
            self.config.from_pyfile(config)
        elif isinstance(config, dict):
            self.config.from_mapping(**config)
        else:
            self.config.from_object(config)
        return self

    def make_config(self, instance_relative=False):
        """Used to create the config attribute by the Flask constructor.
        The `instance_relative` parameter is passed in from the constructor
        of Flask (there named `instance_relative_config`) and indicates if
        the config should be relative to the instance path or the root path
        of the application.
        """
        root_path = self.root_path
        if instance_relative:
            root_path = self.instance_path
        return self.config_class(root_path, self.default_config)

