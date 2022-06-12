#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math

# Define publishers
odom_x_pubs = rospy.Publisher(
    "/cartpole/odom_x_joint_effort_controller/command", Float64, queue_size=10
)
odom_y_pubs = rospy.Publisher(
    "/cartpole/odom_y_joint_effort_controller/command", Float64, queue_size=10
)

if __name__ == "__main__":
    rospy.init_node("joint_vel_command", anonymous=True)

    r = rospy.Rate(100)  # publish msg with 100 Hz
    k = 1
    f = 1

    t = 0
    while not rospy.is_shutdown():
        odom_x_effort = k * math.sin(2 * math.pi * f * t)
        odom_x_pubs.publish(odom_x_effort)

        odom_y_effort = k * math.cos(2 * math.pi * f * t)
        odom_y_pubs.publish(odom_y_effort)

        t += 0.01
        r.sleep()
