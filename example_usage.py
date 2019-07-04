try:
    import rospy
    import roslaunch
    import rosgraph
    import rosnode
except ImportError:  # if ROS environment is not setup, we emulate it.
    print("!!! Original import failed. Attempting Pyros-Setup...")
    import pyros_setup
    pyros_setup.configurable_import().configure('mysetup.cfg').activate()  # this will use mysetup.cfg from pyros-setup instance folder

    import rospy
    import roslaunch
    import rosgraph
    import rosnode

print("rospy from: " + rospy.__file__)
print("roslaunch from: " + roslaunch.__file__)
print("rosgraph from: " + rosgraph.__file__)
print("rosnode from: " + rosnode.__file__)
