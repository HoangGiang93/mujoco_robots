#!/usr/bin/env python

from ntpath import join
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_names = [
    'wheel_front_left_joint',
    'wheel_back_left_joint',
    'wheel_front_right_joint',
    'wheel_back_right_joint',
    'ur5_shoulder_pan_joint',
    'ur5_shoulder_lift_joint',
    'ur5_elbow_joint',
    'ur5_wrist_1_joint',
    'ur5_wrist_2_joint',
    'ur5_wrist_3_joint',
    'gripper_joint'
    ]

pubs = {}
for i in range(4):
    pubs[joint_names[i]] = rospy.Publisher(joint_names[i] + '_velocity_controller/command', Float64, queue_size=10)
for i in range(4,10):
    pubs[joint_names[i]] = rospy.Publisher(joint_names[i] + '_position_controller/command', Float64, queue_size=10)
i = 10
pubs[joint_names[i]] = rospy.Publisher(joint_names[i] + '_effort_controller/command', Float64, queue_size=10)

def get_joint_state_cb(data: JointState):
    for i in range(len(data.name)):
        joint_name = data.name[i]
        position = data.position[i]
        if joint_name in pubs:
            pubs[joint_name].publish(position)
            if joint_name == 'gripper_joint':
                position -= 0.06
                position *= 2000
                pubs[joint_name].publish(position)

if __name__ == '__main__':
    rospy.init_node('command_publisher', anonymous=True)
    rospy.Subscriber('desired_joint_states', JointState, get_joint_state_cb, queue_size=10)
    rospy.spin()
