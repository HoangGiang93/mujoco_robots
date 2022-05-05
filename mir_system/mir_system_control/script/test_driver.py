#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math

rotation_joint_names = [
    "fl_caster_rotation_joint",
    "fr_caster_rotation_joint",
    "bl_caster_rotation_joint",
    "br_caster_rotation_joint",
]

rotation_pubs = [
    rospy.Publisher(
        "/mir/" + rotation_joint_names[i] + "_position_controller/command",
        Float64,
        queue_size=10,
    )
    for i in range(4)
]

wheel_joint_names = [
    "fl_caster_wheel_joint",
    "fr_caster_wheel_joint",
    "bl_caster_wheel_joint",
    "br_caster_wheel_joint",
]

wheel_pubs = [
    rospy.Publisher(
        "/mir/" + wheel_joint_names[i] + "_velocity_controller/command",
        Float64,
        queue_size=10,
    )
    for i in range(4)
]

if __name__ == "__main__":
    rospy.init_node("test_driver", anonymous=True)
    r = rospy.Rate(1)
    k = 1
    v = math.pi / 6
    while not rospy.is_shutdown():
        for i in range(len(rotation_pubs)):
            rotation_pubs[i].publish(0)
        for i in range(len(wheel_pubs)):
            wheel_pubs[i].publish(k * v)
        k = -k

        r.sleep()
