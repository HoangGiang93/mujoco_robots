#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
pubs = [rospy.Publisher('/ur5/' + joint_names[i] + '_velocity_controller/command', Float64, queue_size=10) for i in range(6)]

if __name__ == '__main__':
    rospy.init_node('joint_vel_command', anonymous=True)
    r = rospy.Rate(1)
    k = 1
    v = math.pi/6
    while(not rospy.is_shutdown()):
        for i in range(len(pubs)):
            pubs[i].publish(k * v)
        k = -k
            
        r.sleep()