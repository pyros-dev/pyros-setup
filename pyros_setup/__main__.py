from __future__ import absolute_import
from __future__ import print_function

import sys


def main():
    if len(sys.argv) > 1 and  sys.argv[1] == '--pytest':
        import pytest  # import only if needed
        errno = pytest.main(['--pyargs', 'pyros_setup'] + sys.argv[2:])
        sys.exit(errno)
    else:
        print("To validate your pyros_setup installation, use : pyros_setup --pytest [<custom pytest arguments>]")


if __name__ == "__main__":
    main()

# TODO : improve self test commands (use click if needed)
