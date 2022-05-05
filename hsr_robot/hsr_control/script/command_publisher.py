#!/usr/bin/env python

from ntpath import join
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_names = [
    "arm_flex_joint",
    "arm_lift_joint",
    "arm_roll_joint",
    "head_pan_joint",
    "head_tilt_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
    "hand_motor_joint",
    "hand_l_spring_proximal_joint",
    "hand_r_spring_proximal_joint",
]

pubs = {}
for i in range(len(joint_names)):
    pubs[joint_names[i]] = rospy.Publisher(
        joint_names[i] + "_position_controller/command", Float64, queue_size=10
    )


def get_joint_state_cb(data):
    for i in range(len(data.name)):
        joint_name = data.name[i]
        position = data.position[i]
        if joint_name in pubs:
            pubs[joint_name].publish(position)


if __name__ == "__main__":
    rospy.init_node("command_publisher", anonymous=True)
    rospy.Subscriber(
        "desired_joint_states", JointState, get_joint_state_cb, queue_size=10
    )
    rospy.spin()
