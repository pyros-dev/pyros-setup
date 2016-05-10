# -*- coding: utf-8 -*-
"""
    pyros_setup.packagebound
    ~~~~~~~~~~~~~

    Implements a parentclass to be able to access package relative resources easily.
    Inspired from Flask.helpers
"""

import os
import pkgutil
import sys
from threading import RLock

from .helpers import get_root_path, locked_cached_property, find_package


class PackageBound(object):

    def __init__(self, import_name, instance_path=None, instance_relative_config=True, root_path=None):

        #: The name of the package or module.  Do not change this once
        #: it was set by the constructor.
        self.import_name = import_name

        if root_path is None:
            root_path = get_root_path(self.import_name)

        #: Where is the app root located?
        self.root_path = root_path

        if instance_path is None:
            instance_path = self.auto_find_instance_path()
        elif not os.path.isabs(instance_path):
            raise ValueError('If an instance path is provided it must be '
                             'absolute. A relative path was given instead.')

        #: Holds the path to the instance folder.
        self.instance_path = instance_path

    @locked_cached_property
    def name(self):
        """The name of the application.  This is usually the import name
        with the difference that it's guessed from the run file if the
        import name is main.  This name is used as a display name when
        Flask needs the name of the application.  It can be set and overridden
        to change the value.
        """
        if self.import_name == '__main__':
            fn = getattr(sys.modules['__main__'], '__file__', None)
            if fn is None:
                return '__main__'
            return os.path.splitext(os.path.basename(fn))[0]
        return self.import_name

    def open_resource(self, resource, mode='rb'):
        """Opens a resource from the application's resource folder.  To see
        how this works, consider the following folder structure::
            /myapplication.py
            /schema.sql
            /static
                /style.css
            /templates
                /layout.html
                /index.html
        If you want to open the :file:`schema.sql` file you would do the
        following::
            with app.open_resource('schema.sql') as f:
                contents = f.read()
                do_something_with(contents)
        :param resource: the name of the resource.  To access resources within
                         subfolders use forward slashes as separator.
        :param mode: resource file opening mode, default is 'rb'.
        """
        if mode not in ('r', 'rb'):
            raise ValueError('Resources can only be opened for reading')
        return open(os.path.join(self.root_path, resource), mode)

    def auto_find_instance_path(self):
        """Tries to locate the instance path if it was not provided to the
        constructor of the application class.  It will basically calculate
        the path to a folder named ``instance`` next to your main file or
        the package.
        """
        prefix, package_path = find_package(self.import_name)
        if prefix is None:
            return os.path.join(package_path, 'instance')
        return os.path.join(prefix, 'var', self.name + '-instance')
