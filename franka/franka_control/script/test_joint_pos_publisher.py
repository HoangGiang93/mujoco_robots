#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math

pubs = [rospy.Publisher('/panda_arm/panda_joint' + str(i) + '_position_controller/command', Float64, queue_size=10) for i in range(1,8)]

if __name__ == '__main__':
    rospy.init_node('joint_position_command', anonymous=True)
    r = rospy.Rate(1000)
    f = 0.5
    while(not rospy.is_shutdown()):
        t = rospy.Time.now().to_sec()
        
        for i in range(len(pubs)):
            pubs[i].publish(math.sin(2 * math.pi * f * t))
            
        r.sleep()