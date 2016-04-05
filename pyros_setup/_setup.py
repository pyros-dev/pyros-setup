import logging
import os
import types

import sys

from .helpers import find_package, get_root_path, locked_cached_property
from .config import Config

import warnings


# from http://code.activestate.com/recipes/391367-deprecated/
def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emmitted
    when the function is used."""
    def newFunc(*args, **kwargs):
        warnings.warn("Call to deprecated function %s." % func.__name__,
                      category=DeprecationWarning)
        return func(*args, **kwargs)
    newFunc.__name__ = func.__name__
    newFunc.__doc__ = func.__doc__
    newFunc.__dict__.update(func.__dict__)
    return newFunc


# Metaclass example from http://stackoverflow.com/questions/674304/pythons-use-of-new-and-init
# maybe useful ?
class Singleton(type):
    def __init__(self, *args, **kwargs):
        super(Singleton, self).__init__(*args, **kwargs)
        self.__instance = None

    def __call__(self, *args, **kwargs):
        if self.__instance is None:
            self.__instance = super(Singleton, self).__call__(*args, **kwargs)
        return self.__instance


# class to allow (potentially infinite) delayed conditional import.
# this way it can work with or without preset environment
class _PyrosSetup(types.ModuleType):
    config_class = Config

    #: Default configuration parameters.
    default_config = {
        # pretty standard, but actually useful ?
        'DEBUG': False,
        'TESTING': False,
        # required
        'DISTRO': 'indigo',
        'WORKSPACES': [],
    }

    def __init__(self, import_name=None, instance_path=None, instance_relative_config=True, root_path=None):
        super(_PyrosSetup, self).__init__('pyros_setup', 'Small setup package to dynamically interface with ROS')

        #: The name of the package or module.  Do not change this once
        #: it was set by the constructor.
        self.import_name = import_name or 'pyros_setup'

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

        #: The configuration dictionary as :class:`Config`.  This behaves
        #: exactly like a regular dictionary but supports additional methods
        #: to load a config from files.
        self.config = self.make_config(instance_relative_config)

    def activate(self):
        # TODO : use imp module, or some more fancy stuff (PyMacro style implementation)
        # TODO : put that in context to allow deactivation...
        # The actual trick
        try:
            import rospy  # early except to prevent unintentional workaround in all modules here
            from .ros_utils import get_master, get_ros_home
        except ImportError:
            from .ros_setup import ROS_emulate_setup
            # we want this to expect in case of bad config, because default_config has to have these fields.
            ROS_emulate_setup(self.config['DISTRO'], *self.config['WORKSPACES'])
            try:
                import rospy
                from .ros_utils import get_master, get_ros_home
            except ImportError:
                logging.warn("rospy not found. ROS setup failed !")
                raise

        # members simulating a usual imported module
        self.get_master = get_master
        self.get_ros_home = get_ros_home

        # CAREFUL this doesn't work sometimes (had problem when using from celery bootstep...)
        sys.modules['pyros_setup'] = self
        return self

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

    # encapsulating local imports to delay them until ROS setup is done
    @classmethod
    @deprecated
    def delayed_import(cls, distro=None, *workspaces):

        # building a one time config for bwcompat purpose
        cfg = cls.default_config
        cfg['DISTRO'] = distro or 'indigo'
        cfg['WORKSPACES'] = workspaces
        module_redirect = cls()
        module_redirect.config.from_object(cfg)
        module_redirect.activate()

        return module_redirect

    @classmethod
    @deprecated
    def delayed_import_auto(cls, distro=None, base_path=None):
        import os

        distro = distro or 'indigo'
        # default basepath working if pyros-setup is directly cloned in your workspace
        # This file think it is in devel/lib/python2.7/dist-packages/pyros_setup for some reason...
        # NOTE : maybe the default here is a bad idea, making it simpler than it should be for the user...
        base_path = base_path or os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..')

        # using this for bwcompat purpose only. now seems like a bad idea after all.
        from .ros_setup import ROS_find_workspaces
        workspaces = ROS_find_workspaces(distro, base_path)

        return cls.delayed_import()

    #
    # New configuration style, inspired by flask
    #

    @classmethod
    def delayed_import_config(cls, config_file=None, import_name=None, instance_path=None, instance_relative_config=True, root_path=None):
        # we return a relay of imported names, accessible the same way a direct import would be.
        module_redirect = cls(
            import_name=import_name,
            instance_path=instance_path,
            instance_relative_config=instance_relative_config,
            root_path=root_path
        )
        module_redirect.config.from_pyfile(config_file)
        return module_redirect

    def auto_find_instance_path(self):
        """Tries to locate the instance path if it was not provided to the
        constructor of the application class.  It will basically calculate
        the path to a folder named ``instance`` next to your main file or
        the package.
        .. versionadded:: 0.8
        """
        prefix, package_path = find_package(self.import_name)
        if prefix is None:
            return os.path.join(package_path, 'instance')
        return os.path.join(prefix, 'var', self.name + '-instance')

    def make_config(self, instance_relative=False):
        """Used to create the config attribute by the Flask constructor.
        The `instance_relative` parameter is passed in from the constructor
        of Flask (there named `instance_relative_config`) and indicates if
        the config should be relative to the instance path or the root path
        of the application.
        .. versionadded:: 0.8
        """
        root_path = self.root_path
        if instance_relative:
            root_path = self.instance_path
        return self.config_class(root_path, self.default_config)

