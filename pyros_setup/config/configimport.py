# -*- coding: utf-8 -*-
from __future__ import absolute_import

import importlib
import logging
import types
import sys
import six

from .confighandler import ConfigHandler


# class to allow (potentially infinite) delayed conditional import.
# this way it can work with or without preset environment
class ConfigImport(types.ModuleType):

    def __init__(self, import_name, import_desc, relay_import_dict, fix_imports,
                 instance_path=None, instance_relative_config=True, root_path=None, default_config=None):
        super(ConfigImport, self).__init__(import_name, import_desc)

        # we delegate config related behavior (including defaults)
        self.config_handler = ConfigHandler(
            import_name,
            instance_path=instance_path,
            instance_relative_config=instance_relative_config,
            root_path=root_path,
            default_config=default_config,
        )

        #: relay_import_dict is a dictionary of the following form:
        #: {
        #:  'symbol_name_1': 'absolute.module.name',
        #:  'symbol_name_2': ('absolute.module.name', 'unused'),
        #:  'symbol_name_3': ('.relative.module.name', 'package_anchor'),
        #: }
        #: Its content will be passed to importlib.import_module() to attempt importing the module
        self.relay_import_dict = relay_import_dict

        assert callable(fix_imports)
        #: _fix_imports should be set to a callable.
        #: it will be used to attempt fixing the environment if needed before retrying importing...
        self._fix_imports = fix_imports

    @property
    def name(self):
        return self.config_handler.name

    @property
    def import_name(self):
        return self.config_handler.import_name

    @property
    def config(self):
        return self.config_handler.config

    @property
    def instance_path(self):
        return self.config_handler.instance_path

    @property
    def instance_relative_config(self):
        return self.config_handler.instance_relative_config

    @property
    def root_path(self):
        return self.config_handler.root_path

    def configure(self, config=None):
        # We default to using a config file named after the import_name:
        config = config or self.name + '.cfg'
        self.config_handler.configure(config)
        return self

    def activate(self):
        # TODO : put that in context to allow deactivation...

        symbols = {}

        # The actual trick
        try:
            # Try in case we don't need the fix
            for n, m in six.iteritems(self.relay_import_dict):
                if isinstance(m, tuple):
                    symbols[n] = importlib.import_module(m[0], m[1])
                else:
                    symbols[n] = importlib.import_module(m)
        except ImportError:
            # fix because it seems we need it
            self._fix_imports()
            # Try again and let it break explicitely
            for n, m in six.iteritems(self.relay_import_dict):
                if isinstance(m, tuple):
                    symbols[n] = importlib.import_module(m[0], m[1])
                else:
                    symbols[n] = importlib.import_module(m)

        for n in symbols.keys():
            # establishing internal relays:
            setattr(self, n, symbols[n])

        # Main relay
        # CAREFUL this doesn't work sometimes (had problem when using from celery bootstep...)
        sys.modules[self.import_name] = self
        return self
