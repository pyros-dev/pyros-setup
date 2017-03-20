
# A very special ROS hack that emulate a ros environment when imported from python
# Useful for using all python tools ( tests, IDE, etc. ) without having to do all the ROS setup beforehand

import sys
import os
import site
import logging
import traceback
import pkg_resources

from ._version import __version__

# create logger
_logger = logging.getLogger(__name__)
# and let it propagate to parent logger, or other handler
# the user of pyros-config should configure handlers


# Functions needed to find our ros code from different possible run environments,
#  even when ROS has not been setup previously ( source setup, roslaunch, rostest ).
# This is especially useful when debugging directly from Python IDE or so.
#
# Here we need to setup an environment that can support different types of run:
#
# - nosetests
# - pycharm UI test runs (with easy debug)
# - rostest  => not supported yet
#    rostest try to run the python code as executable -> needs more hacks to get modules from source
#    nosetest has special import code that discover the package modules nicely
#    Still needs more investigation in why rostest doesnt work / works differently to nose.
#    Maybe not worth it.
# - TODO : tox + py.test (benchmark)


def ROS_setup_rosdistro_env(default_distro=None):

    # Note : running setup.bash script is not really better.
    # We need to transpose all useful variables to our environment, not only PYTHONPATH.
    # There would quite a lot of code involved to do this anyway.

    # TODO : investigate how to import and use the devel/_setup_util.py from here...
    # The goal is to minimize code and simplify maintenance.

    default_distro = default_distro or 'indigo'
    # Setting env var like ROS would
    # TODO : find the proper place in ros where this is set and use it instead
    if os.environ.get('ROS_DISTRO', None) is None:
                os.environ['ROS_DISTRO'] = default_distro

    distro = os.environ['ROS_DISTRO']
    if os.environ.get('ROS_ROOT', None) is None:
                os.environ['ROS_ROOT'] = '/opt/ros/' + distro + '/share/ros'

    if os.environ.get('ROS_PACKAGE_PATH', None) is None:
                os.environ['ROS_PACKAGE_PATH'] = ':'.join(['/opt/ros/' + distro + '/share',
                                                           '/opt/ros/' + distro + '/stacks'])

    if os.environ.get('ROS_MASTER_URI', None) is None:
                os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'

    if os.environ.get('ROS_ETC_DIR', None) is None:
                os.environ['ROS_ETC_DIR'] = '/opt/ros/' + distro + '/etc/ros'

    # we return here the workspace for the distro
    return '/opt/ros/' + distro


def ROS_setup_ros_package_path(workspace):

    # setting cmake prefix path - rosout needs this
    _logger.debug("Checking {0} exists and not in {1}".format(workspace, os.environ.get("CMAKE_PREFIX_PATH", [])))
    if os.path.exists(workspace) and workspace not in os.environ.get("CMAKE_PREFIX_PATH", []):
        _logger.warning("Prepending path {workspace} to CMAKE_PREFIX_PATH".format(workspace=workspace))
        os.environ["CMAKE_PREFIX_PATH"] = workspace + ':' + os.environ.get("CMAKE_PREFIX_PATH", '')

    # prepending current path for ros package discovery
    _logger.debug("Checking {0} exists and is named 'devel'".format(workspace))
    if os.path.basename(workspace) == 'devel':  # special case of devel -> we can find src
        src_path = os.path.join(os.path.dirname(workspace), 'src')
        _logger.debug("Checking {0} exists".format(src_path))
        if src_path is not None and os.path.exists(src_path):
            _logger.warning("Prepending path {workspace_src} to ROS_PACKAGE_PATH".format(workspace_src=src_path))
            os.environ['ROS_PACKAGE_PATH'] = src_path + ':' + os.environ['ROS_PACKAGE_PATH']

    else:
        stacks_path = os.path.join(workspace, 'stacks')
        _logger.debug("Checking {0} exists".format(stacks_path))
        if stacks_path is not None and os.path.exists(stacks_path):
            _logger.warning("Prepending path {workspace_stacks} to ROS_PACKAGE_PATH".format(workspace_stacks=stacks_path))
            os.environ['ROS_PACKAGE_PATH'] = stacks_path + ':' + os.environ['ROS_PACKAGE_PATH']

        share_path = os.path.join(workspace, 'share')
        _logger.debug("Checking {0} exists".format(share_path))
        if share_path is not None and os.path.exists(share_path):
            _logger.warning("Prepending path {workspace_share} to ROS_PACKAGE_PATH".format(workspace_share=share_path))
            os.environ['ROS_PACKAGE_PATH'] = share_path + ':' + os.environ['ROS_PACKAGE_PATH']


def ROS_setup_ospath(workspace):

    binpath = os.path.join(workspace, 'bin')
    if binpath is not None and os.path.exists(binpath):  # note: even if it already exist in PATH we add it again
        _logger.warning("Prepending path {binpath} to PATH".format(binpath=binpath))
        os.environ["PATH"] = binpath + ':' + os.environ.get("PATH", '')


def ROS_setup_ldlibrarypath(workspace):
    lib_path = os.path.join(workspace, 'lib')
    libarch_path = os.path.join(workspace, 'lib', 'x86_64-linux-gnu')

    if libarch_path is not None and os.path.exists(libarch_path):  # note: even if it already exist in PATH we add it again
        _logger.warning("Prepending path {libarch_path} to LD_LIBRARY_PATH".format(libarch_path=libarch_path))
        os.environ["LD_LIBRARY_PATH"] = libarch_path + ':' + os.environ.get("LD_LIBRARY_PATH", '')

    if lib_path is not None and os.path.exists(lib_path):  # note: even if it already exist in PATH we add it again
        _logger.warning("Prepending path {lib_path} to LD_LIBRARY_PATH".format(lib_path=lib_path))
        os.environ["LD_LIBRARY_PATH"] = lib_path + ':' + os.environ.get("LD_LIBRARY_PATH", '')


def ROS_setup_pkgconfigpath(workspace):
    libpkgconfig_path = os.path.join(workspace, 'lib', 'pkgconfig')
    libarchpkgconfig_path = os.path.join(workspace, 'lib', 'x86_64-linux-gnu', 'pkgconfig')

    if libarchpkgconfig_path is not None and os.path.exists(libarchpkgconfig_path):  # note: even if it already exist in PATH we add it again
        _logger.warning("Prepending path {libarchpkgconfig_path} to PKG_CONFIG_PATH".format(libarchpkgconfig_path=libarchpkgconfig_path))
        os.environ["PKG_CONFIG_PATH"] = libarchpkgconfig_path + ':' + os.environ.get("PKG_CONFIG_PATH", '')

    if libpkgconfig_path is not None and os.path.exists(libpkgconfig_path):  # note: even if it already exist in PATH we add it again
        _logger.warning("Prepending path {libpkgconfig_path} to PKG_CONFIG_PATH".format(libpkgconfig_path=libpkgconfig_path))
        os.environ["PKG_CONFIG_PATH"] = libpkgconfig_path + ':' + os.environ.get("PKG_CONFIG_PATH", '')


# TODO : check if we can use roslib.load_manifest for all this
def ROS_setup_pythonpath(workspace):

    # TODO : support python3 using checks like : sys.version_info >= (2, 7)
    # for indigo, python3 is better supported via virtual envs to not conflict with system install.
    package_paths = [
        os.path.join(workspace, 'lib', 'python2.7', 'dist-packages'),  # default for catkin
        os.path.join(workspace, 'lib', 'python2.7', 'site-packages'),  # catkin_pip can create this in workspaces and we need to be able to access it
    ]

    # since ROS logic with package path is *incompatible* with python / venv logic
    # inspired from python site module (getting rid of known_path)
    def readdsitedir(sitedir):
        """Add 'sitedir' argument to sys.path if missing and handle .pth files in
        'sitedir'"""
        sitedir, sitedircase = site.makepath(sitedir)
        print("Re-adding {sitedir} in front of sys.path".format(**locals()))
        if sitedir in sys.path:
            sys.path.remove(sitedir)  # Remove path component
        sys.path.insert(1, sitedir)  # Insert path component in front
        try:
            names = os.listdir(sitedir)
        except os.error:
            return
        dotpth = os.extsep + "pth"
        names = [name for name in names if name.endswith(dotpth)]
        for name in sorted(names):
            readdpackage(sitedir, name)
        return

    # since ROS logic with package path is *incompatible* with python / venv logic
    # inspired from python site module (getting rid of known_path)
    def readdpackage(sitedir, name):
        """Process a .pth file within the site-packages directory:
           For each line in the file, if it doesnt start with 'import ', combine it with sitedir to a path
           and remove that from sys.path.
        """
        # All this code is inspired from site.addpackage
        fullname = os.path.join(sitedir, name)
        try:
            f = open(fullname, "rU")
        except IOError:
            return
        with f:
            for n, line in enumerate(f):
                if line.startswith("#") or line.startswith(("import ", "import\t")):
                    continue
                try:
                    line = line.rstrip()
                    dir, dircase = site.makepath(sitedir, line)
                    # If the path is already there, we remove it (to reinsert it in the proper place)
                    print("Re-adding {dir} in front of sys.path".format(**locals()))
                    if dir in sys.path:
                        sys.path.remove(dir)
                    sys.path.insert(1, dir)
                except Exception as err:
                    print >> sys.stderr, "Error processing line {:d} of {}:\n".format(
                        n + 1, fullname)
                    for record in traceback.format_exception(*sys.exc_info()):
                        for line in record.splitlines():
                            print >> sys.stderr, '  ' + line
                    print >> sys.stderr, "\nRemainder of file ignored"
                    break

    # Note : virtualenvs are a much better solution to this problem,
    # but we have to simulate ROS behavior ( to work with ROS workspaces )

    # setting python path needed only to find ros shell commands (rosmaster)
    pplist = os.environ.get("PYTHONPATH", "").split(":")

    for pp in package_paths:
        if pp is not None and os.path.exists(pp):
            _logger.warning("Prepending path {pp} to sys.path".format(**locals()))

            readdsitedir(pp)

            # similar logic for pythonpath as with readdsitedir
            if pp in pplist:
                pplist.remove(pp)
            pplist.insert(1, pp)

            # making sure our logic works for namespace packages as well
            pkg_resources.fixup_namespace_packages(pp)  # ensure that message subpackage is included

    os.environ["PYTHONPATH"] = ':'.join(pplist)


    # Only this method is enough to fix the python import issues.
    # However it is expected that the whole ROS environment is setup before importing rospy, to avoid unknown issues.

    #TODO : check if we can use http://setuptools.readthedocs.io/en/latest/pkg_resources.html#overview
    #TODO : plugins https://www.reddit.com/r/Python/comments/4jwu33/importing_python_modules_with_pkg_resources/
    # To make checking for dependencies easier and less error prone.


def ROS_find_workspaces(distro, base_path):
    """
    Helper function to find your workspaces for you.
    :param distro: distribution name
    :param base_path: the base path of your workspace ( should contain devel/ and src/ folder at least )
    :return: list of workspaces path, directly usable with ROS_emulate_setup
    """

    cmake_env_var = "CMAKE_PREFIX_PATH"
    # if CMAKE_PREFIX_PATH is set we can find everything else
    # useful to fix a broken setup ( can it really happen ? )

    cmake_env_path = os.environ.get(cmake_env_var, '')
    workspace_paths = cmake_env_path.split(':')

    if not cmake_env_path:  # empty string : we failed looking for it in environment
        install_ws = os.path.abspath(os.path.join(base_path, 'install'))
        devel_ws = os.path.abspath(os.path.join(base_path, 'devel'))
        workspace_paths = []
        # setting cmake prefix path - rosout needs this
        for k, p in zip(['devel', 'install'], [devel_ws, install_ws]):
            if os.path.exists(p) and p not in os.environ.get("CMAKE_PREFIX_PATH", []):
                _logger.warning("Appending {key} space to CMake prefix path".format(key=k))
                os.environ["CMAKE_PREFIX_PATH"] = p + ':' + os.environ.get("CMAKE_PREFIX_PATH", '')
                workspace_paths[:0] = [p]  # prepending

    return tuple(workspace_paths)


def ROS_emulate_setup(distro=None, *workspaces):
    """
    :param distro: ROS distribution for which you want to emulate the setup.bash behavior
    :param workspaces: workspace arguments, ordered from overlay to underlay :
        devel_path ( usual workspace development setup )
         OR
        install_path ( usual install workspace setup )
         OR
        over_devel_path, under_devel_path ( usual workspace development with underlays setup )
    :return:
    """
    _logger.warning(" => Pyros_setup v{0} Emulating ROS setup now for distro {1} and workspaces {2}".format(__version__, distro, workspaces))

    distro = distro or 'indigo'  # TODO : investigate if we should use /usr/bin/rosversion to determine default ?
    distro_path = ROS_setup_rosdistro_env(default_distro=distro)

    # adding distro_path to the workspace list
    workspaces = list(workspaces) + [distro_path]

    # we need to reverse the order because we prepend in all these functions
    for w in reversed(workspaces):
        if not os.path.exists(w):
            _logger.warning("Configured workspace {w} not found. Please double check your configuration. Skipping...".format(**locals()))
        else:
            ROS_setup_ros_package_path(w)
            ROS_setup_ospath(w)
            ROS_setup_ldlibrarypath(w)
            ROS_setup_pkgconfigpath(w)
            ROS_setup_pythonpath(w)

    _logger.warning(" => ROS setup emulation done.")
