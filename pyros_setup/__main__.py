from __future__ import absolute_import
from __future__ import print_function

import sys


def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == '--pytest':
            import pytest  # import only if needed
            errno = pytest.main(['-s', '--pyargs', 'pyros_setup'] + sys.argv[2:])
            sys.exit(errno)
        elif sys.argv[1] == '--version':
            import pyros_setup  # import only if needed
            # following pip version output format
            print("pyros_setup {0} from {1} (python {2})".format(pyros_setup.__version__, pyros_setup.__file__, sys.version[:3]))
            return
        elif sys.argv[1] == '--genconfig':
            import pyros_setup  # import only if needed
            pyros_setup.configurable_import().generate_default_config()
            return
        elif sys.argv[1] == '--help':
            print("Pyros_setup is a tool to help you manipulate python environment setup.")
            print("It is especially useful with ROS and other environments that rely on system python with PYTHONPATH modifications.")
            print("Usage: pyros_setup [--pytest | --version | --genconfig | --help]")
        else:
            print("Usage: pyros_setup [--pytest | --version | --genconfig | --help]")

    else:
        print("To validate your pyros_setup installation, use : pyros_setup --pytest [<custom pytest arguments>]")
        print("To check your pyros_setup version, use : pyros_setup --version")
        print("To get help, use : pyros_setup --help")


if __name__ == "__main__":
    main()

# TODO : improve self test commands (use click if needed)
