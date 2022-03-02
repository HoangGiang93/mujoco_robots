#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
pubs = [rospy.Publisher('/ur5/' + joint_names[i] + '_position_controller/command', Float64, queue_size=10) for i in range(6)]

if __name__ == '__main__':
    rospy.init_node('joint_position_command', anonymous=True)
    r = rospy.Rate(1000)
    f = 0.5
    while(not rospy.is_shutdown()):
        t = rospy.Time.now().to_sec()
        
        for i in range(len(pubs)):
            pubs[i].publish(math.sin(2 * math.pi * f * t))
            
        r.sleep()