# -*- coding: utf-8 -*-
from __future__ import absolute_import


# encapsulating local imports to delay them until ROS setup is done
def delayed_import():
    try:
        import rospy
        from .ros_utils import ROS_Master
    except ImportError:
        from .ros_setup import ROS_emulate_setup
        ROS_emulate_setup()
        import rospy
        from .ros_utils import ROS_Master

    # we return a dict of imported names  the same way import would do
    return {
        'ROS_Master': ROS_Master
    }

__all__ = [
    'delayed_import',
]