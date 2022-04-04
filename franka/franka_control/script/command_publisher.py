#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

pubs = [rospy.Publisher('panda_joint' + str(i) + '_position_controller/command', Float64, queue_size=10) for i in range(1,8)]

def get_joint_state_cb(data):
    for i in range(len(pubs)):
        pubs[i].publish(data.position[i])

if __name__ == '__main__':
    rospy.init_node('command_publisher', anonymous=True)
    rospy.Subscriber('desired_joint_states', JointState, get_joint_state_cb, queue_size=10)
    rospy.spin()
