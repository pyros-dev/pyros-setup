
def pytest_addoption(parser):
    parser.addoption("--distro", action="store", default=None,
        help="distro: indigo or jade or kinetic")
