import logging
import sys
import types
import warnings

from .config import PackageBoundConfigHandler


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
# class Singleton(type):
#     def __init__(self, *args, **kwargs):
#         super(Singleton, self).__init__(*args, **kwargs)
#         self.__instance = None
#
#     def __call__(self, *args, **kwargs):
#         if self.__instance is None:
#             self.__instance = super(Singleton, self).__call__(*args, **kwargs)
#         return self.__instance


# class to allow (potentially infinite) delayed conditional import.
# this way it can work with or without preset environment
class _PyrosSetup(types.ModuleType):

    #: Default configuration parameters.
    default_config = {
        'DISTRO': 'indigo',
        'WORKSPACES': [],
    }

    def __init__(self, import_name=None, instance_path=None, instance_relative_config=True, root_path=None):
        super(_PyrosSetup, self).__init__('pyros_setup', 'Small setup package to dynamically interface with ROS')

        import_name = import_name or 'pyros_setup'

        self.config_handler = PackageBoundConfigHandler(
            import_name,
            instance_path=instance_path,
            instance_relative_config=instance_relative_config,
            root_path=root_path,
            default_config=self.default_config
        )

    @property
    def name(self):
        return self.config_handler.name

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

    def configure_from_pyfile(self, config_file=None):
        # Setup desired configuration
        self.config.from_pyfile(config_file)
        return self

    def configure_from_object(self, config_object=None):
        # Setup desired configuration
        self.config.from_object(config_object)
        return self

    def configure_from_mapping(self, *args, **kwargs):
        # Setup desired configuration
        self.config.from_mapping(*args, **kwargs)
        return self

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
    def delayed_import_new(cls, import_name=None, instance_path=None, instance_relative_config=True, root_path=None):
        # we return a relay of imported names, accessible the same way a direct import would be.
        module_redirect = cls(
            import_name=import_name,
            instance_path=instance_path,
            instance_relative_config=instance_relative_config,
            root_path=root_path
        )
        return module_redirect


