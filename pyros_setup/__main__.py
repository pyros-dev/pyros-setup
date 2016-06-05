from __future__ import absolute_import
from __future__ import print_function

import sys


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--pytest':
        import pytest  # import only if needed
        errno = pytest.main(sys.argv[2:])
        sys.exit(errno)


if __name__ == "__main__":
    main()

