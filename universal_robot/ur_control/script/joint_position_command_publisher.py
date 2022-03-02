#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
pubs = [rospy.Publisher(joint_names[i] + '_position_controller/command', Float64, queue_size=10) for i in range(6)]

def get_joint_state_cb(data):
    for i in range(len(pubs)):
        pubs[i].publish(data.position[i])

if __name__ == '__main__':
    rospy.init_node('joint_position_command', anonymous=True)
    rospy.Subscriber('desired_joint_states', JointState, get_joint_state_cb, queue_size=10)
    rospy.spin()
