
def pytest_addoption(parser):
    # Hack to workaround : https://github.com/pytest-dev/pytest/issues/1889
    #print("options detected : {0}".format([name for opt in parser._anonymous.options for name in opt.names()]))
    #conflict = set("--distro").intersection(name for opt in parser._anonymous.options for name in opt.names())
    #print("conflict : {0}".format(conflict))
    #if not conflict:
        parser.addoption("--distro", action="store", default=None,
            help="distro: indigo or jade or kinetic")
