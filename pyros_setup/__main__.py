from __future__ import absolute_import
from __future__ import print_function

import sys


def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == '--pytest':
            import pytest  # import only if needed
            errno = pytest.main(['--pyargs', 'pyros_setup'] + sys.argv[2:])
            sys.exit(errno)
        if sys.argv[1] == '--version':
            import pyros_setup  # import only if needed
            print("pyros_setup from {0} version {1}".format(pyros_setup.__file__, pyros_setup.__version__))
            return
        if sys.argv[1] == '--help':
            print("Pyros_setup is a tool to help you manipulate python environment setup.")
            print("It is especially useful with ROS and other environments that rely on system python with PYTHONPATH modifications.")
            print("Usage: pyros_setup [--pytest | --version | --help]")
    else:
        print("To validate your pyros_setup installation, use : pyros_setup --pytest [<custom pytest arguments>]")
        print("To check your pyros_setup version, use : pyros_setup --version")
        print("To get help, use : pyros_setup --help")


if __name__ == "__main__":
    main()

# TODO : improve self test commands (use click if needed)
