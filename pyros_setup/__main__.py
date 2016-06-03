from __future__ import absolute_import
from __future__ import print_function

import sys
import pytest

if __name__ == "__main__":

    if len(sys.argv) > 1 and sys.argv[1] == '--pytest':
        sys.exit(pytest.main(sys.argv[2:]))

