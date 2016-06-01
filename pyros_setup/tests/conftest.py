
def pytest_addoption(parser):
    parser.addoption("--distro", action="store", default="indigo",
        help="distro: indigo or jade")
