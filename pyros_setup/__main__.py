from __future__ import absolute_import
from __future__ import print_function

import os
import sys

# logging configuration should be here to not be imported by python users of pyros_setup.
# only used from command line

import logging.config
# Setting up logging for this test
logging.config.dictConfig(
    {
        'version': 1,
        'formatters': {
            'verbose': {
                'format': '%(levelname)s %(asctime)s %(module)s %(process)d %(thread)d %(message)s'
            },
            'simple': {
                'format': '%(levelname)s %(name)s:%(message)s'
            },
        },
        'handlers': {
            'null': {
                'level': 'DEBUG',
                'class': 'logging.NullHandler',
            },
            'console': {
                'level': 'DEBUG',
                'class': 'logging.StreamHandler',
                'formatter': 'simple'
            },
        },
        'loggers': {
            'pyros_config': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup.common': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup.indigo': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup.jade': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            },
            'pyros_setup.kinetic': {
                'handlers': ['console'],
                'level': 'INFO',
                'propagate': False,
            }
        }
    }
)


# TODO : use click for cleaner command line arg parsing.
def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == '--config':

            if os.path.exists('/opt/ros/kinetic'):
                import pyros_setup.kinetic  # import only if needed
                pyros_setup.kinetic.configure()  # this will create the configuration if needed
                pyros_setup.kinetic.show_config()
            if os.path.exists('/opt/ros/jade'):
                import pyros_setup.jade  # import only if needed
                pyros_setup.jade.configure()  # this will create the configuration if needed
                pyros_setup.jade.show_config()
            if os.path.exists('/opt/ros/indigo'):
                import pyros_setup.indigo  # import only if needed
                pyros_setup.indigo.configure()  # this will create the configuration if needed
                pyros_setup.indigo.show_config()

        elif sys.argv[1] == '--pytest':
            import pytest  # import only if needed
            errno = pytest.main(['-s', '--pyargs', 'pyros_setup'] + sys.argv[2:])
            sys.exit(errno)
        elif sys.argv[1] == '--version':
            import pyros_setup  # import only if needed
            # following pip version output format
            print("pyros_setup {0} from {1} (python {2})".format(pyros_setup.__version__, pyros_setup.__file__, sys.version[:3]))
            return
        elif sys.argv[1] == '--help':
            print("Pyros_setup is a tool to help you manipulate python environment setup.")
            print("It is especially useful with ROS and other environments that rely on system python with PYTHONPATH modifications.")
            print("Usage: pyros_setup [--config | --pytest | --version | --help]")
        else:
            print("Usage: pyros_setup [--config | --pytest | --version | --help]")

    else:
        print("To validate your pyros_setup installation, use : pyros_setup --pytest [<custom pytest arguments>]")
        print("To check your pyros_setup version, use : pyros_setup --version")
        print("To get help, use : pyros_setup --help")


if __name__ == "__main__":
    main()

# TODO : improve self test commands (use click if needed)
