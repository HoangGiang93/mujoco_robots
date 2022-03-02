#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math

pubs = [rospy.Publisher('/panda_arm/panda_joint' + str(i) + '_velocity_controller/command', Float64, queue_size=10) for i in range(1,8)]

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