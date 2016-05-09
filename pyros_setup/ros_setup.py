
# A very special ROS hack that emulate a ros environment when imported from python
# Useful for using all python tools ( tests, IDE, etc. ) without having to do all the ROS setup beforehand

import sys
import os
import logging
import collections

# Functions needed to find our ros code from different possible run environments,
#  even when ROS has not been setup previously ( source setup, roslaunch, rostest ).
# This is especially useful when debugging directly from Python IDE or so.
#
# Note : We dont want to support all the overlays / underlay things here
# -> only applicable for package on top of core ROS packages.
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

    if os.environ.get('ROS_MASTER_URI', None) is None:
                os.environ['ROS_MASTER_URI'] = '/opt/ros/' + distro + '/etc/ros'

    # we return here the workspace for the distro
    return '/opt/ros/' + distro

def ROS_setup_ros_package_path(workspace):

    # setting cmake prefix path - rosout needs this
    if os.path.exists(workspace) and workspace not in os.environ.get("CMAKE_PREFIX_PATH", []):
        logging.warn("Appending path {workspace} to CMAKE_PREFIX_PATH".format(workspace=workspace))
        os.environ["CMAKE_PREFIX_PATH"] = workspace + ':' + os.environ.get("CMAKE_PREFIX_PATH", '')

    # prepending current path for ros package discovery
    if os.path.basename(workspace) == 'devel':  # special case of devel -> we can find src
        logging.warn("Appending path {workspace_src} to ROS_PACKAGE_PATH".format(workspace_src=os.path.join(os.path.dirname(workspace), 'src')))
        os.environ['ROS_PACKAGE_PATH'] = os.path.join(os.path.dirname(workspace), 'src') + ':' \
                                         + os.environ['ROS_PACKAGE_PATH']
    else:  # TODO : this is a quick fix. investigate this case more
        logging.warn("Appending path {workspace_stacks} to ROS_PACKAGE_PATH".format(workspace_stacks=os.path.join(workspace, 'stacks')))
        logging.warn("Appending path {workspace_share} to ROS_PACKAGE_PATH".format(workspace_share=os.path.join(workspace, 'share')))
        os.environ['ROS_PACKAGE_PATH'] = os.path.join(workspace, 'share') + ':' \
                                         + os.path.join(workspace, 'stacks') + ':' \
                                         + os.environ['ROS_PACKAGE_PATH']



def ROS_setup_ospath(workspace):

    binpath = os.path.join(workspace, 'bin')
    if binpath is not None and os.path.exists(binpath):  # note: even if it already exist in PATH we add it again
        logging.warn("Appending path {binpath} to PATH".format(binpath=binpath))
        os.environ["PATH"] = binpath + ':' + os.environ.get("PATH", '')


def ROS_setup_ldlibrarypath(workspace):
    lib_path = os.path.join(workspace, 'lib')
    libarch_path = os.path.join(workspace, 'lib', 'x86_64-linux-gnu')

    if libarch_path is not None and os.path.exists(libarch_path):  # note: even if it already exist in PATH we add it again
        logging.warn("Appending path {libarch_path} to LD_LIBRARY_PATH".format(libarch_path=libarch_path))
        os.environ["LD_LIBRARY_PATH"] = libarch_path + ':' + os.environ.get("LD_LIBRARY_PATH", '')

    if lib_path is not None and os.path.exists(lib_path):  # note: even if it already exist in PATH we add it again
        logging.warn("Appending path {lib_path} to LD_LIBRARY_PATH".format(lib_path=lib_path))
        os.environ["LD_LIBRARY_PATH"] = lib_path + ':' + os.environ.get("LD_LIBRARY_PATH", '')


def ROS_setup_pkgconfigpath(workspace):
    libpkgconfig_path = os.path.join(workspace, 'lib', 'pkgconfig')
    libarchpkgconfig_path = os.path.join(workspace, 'lib', 'x86_64-linux-gnu', 'pkgconfig')

    if libarchpkgconfig_path is not None and os.path.exists(libarchpkgconfig_path):  # note: even if it already exist in PATH we add it again
        logging.warn("Appending path {libarchpkgconfig_path} to PKG_CONFIG_PATH".format(libarchpkgconfig_path=libarchpkgconfig_path))
        os.environ["PKG_CONFIG_PATH"] = libarchpkgconfig_path + ':' + os.environ.get("PKG_CONFIG_PATH", '')

    if libpkgconfig_path is not None and os.path.exists(libpkgconfig_path):  # note: even if it already exist in PATH we add it again
        logging.warn("Appending path {libpkgconfig_path} to PKG_CONFIG_PATH".format(libpkgconfig_path=libpkgconfig_path))
        os.environ["PKG_CONFIG_PATH"] = libpkgconfig_path + ':' + os.environ.get("PKG_CONFIG_PATH", '')


# TODO : check if we can use roslib.load_manifest for all this
def ROS_setup_pythonpath(workspace):

    package_path = os.path.join(workspace, 'lib', 'python2.7', 'dist-packages')

    if package_path is not None and os.path.exists(package_path):
        logging.warn("Prepending path {package_path} to PYTHONPATH".format(package_path=package_path))
        # Note : virtualenvs are a much better solution to this problem.
        # nevertheless we here try to simulate ROS behavior ( working with workspaces )
        sys.path.insert(1, package_path)
        # setting python path needed only to find ros shell commands (rosmaster)
        os.environ["PYTHONPATH"] = package_path + ':' + os.environ.get("PYTHONPATH", '')

    # Only this method is enough to fix the python import issues.
    # However all ROS environment should be setup before importing rospy due to https://github.com/ros/catkin/issues/767


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
                logging.warn("Appending {key} space to CMake prefix path".format(key=k))
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
    logging.warn(" => Emulating ROS setup now for distro {0} and workspaces {1}".format(distro, workspaces))

    distro = distro or 'indigo'  # TODO : investigate if we should use /usr/bin/rosversion to determine default ?
    distro_path = ROS_setup_rosdistro_env(default_distro=distro)

    # adding distro_path to the workspace list
    workspaces = list(workspaces) + [distro_path]

    # we need to reverse the order because we prepend in all these functions
    for w in reversed(workspaces):
        ROS_setup_ros_package_path(w)
        ROS_setup_ospath(w)
        ROS_setup_ldlibrarypath(w)
        ROS_setup_pkgconfigpath(w)
        ROS_setup_pythonpath(w)

    logging.warn(" => ROS setup emulation done.")
