#!/usr/bin/env python3

import rospy
from control_msgs.msg import GripperCommandActionGoal
from std_msgs.msg import Float64
import copy

left_finger_names = [
    "Index_L_1_Joint",
    "Index_L_2_Joint",
    "Index_L_3_Joint",
    "Middle_L_1_Joint",
    "Middle_L_2_Joint",
    "Middle_L_3_Joint",
    "Ring_L_1_Joint",
    "Ring_L_2_Joint",
    "Ring_L_3_Joint",
    "Pinky_L_1_Joint",
    "Pinky_L_2_Joint",
    "Pinky_L_3_Joint"]

left_thumb_names = [
    "Thumb_L_1_Joint",
    "Thumb_L_2_Joint"]

right_finger_names = [
    "Index_R_1_Joint",
    "Index_R_2_Joint",
    "Index_R_3_Joint",
    "Middle_R_1_Joint",
    "Middle_R_2_Joint",
    "Middle_R_3_Joint",
    "Ring_R_1_Joint",
    "Ring_R_2_Joint",
    "Ring_R_3_Joint",
    "Pinky_R_1_Joint",
    "Pinky_R_2_Joint",
    "Pinky_R_3_Joint"]

right_thumb_names = [
    "Thumb_R_1_Joint",
    "Thumb_R_2_Joint"]

pubs = {}
for joint_name in left_finger_names + left_thumb_names + right_finger_names + right_thumb_names:
    pubs[joint_name] = rospy.Publisher(
        joint_name + '_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)

def left_fingers_control(data: Float64):
    control_msg = GripperCommandActionGoal()
    control_msg.header.seq += 1
    control_msg.header.stamp = rospy.Time().now()
    control_msg.goal_id.stamp = control_msg.header.stamp
    
    for joint_name in left_finger_names:
        control_msg.header.frame_id = joint_name
        control_msg.goal_id.id = str(control_msg.header.seq)
        control_msg.goal.command.position = data.data
        control_msg.goal.command.max_effort = 100
        control_msg_tmp = copy.deepcopy(control_msg)
        pubs[joint_name].publish(control_msg_tmp)

def left_thumb_control(data: Float64):
    control_msg = GripperCommandActionGoal()
    control_msg.header.seq += 1
    control_msg.header.stamp = rospy.Time().now()
    control_msg.goal_id.stamp = control_msg.header.stamp
    
    for joint_name in left_thumb_names:
        control_msg.header.frame_id = joint_name
        control_msg.goal_id.id = str(control_msg.header.seq)
        control_msg.goal.command.position = data.data
        control_msg.goal.command.max_effort = 100
        control_msg_tmp = copy.deepcopy(control_msg)
        pubs[joint_name].publish(control_msg_tmp)

def right_fingers_control(data: Float64):
    control_msg = GripperCommandActionGoal()
    control_msg.header.seq += 1
    control_msg.header.stamp = rospy.Time().now()
    control_msg.goal_id.stamp = control_msg.header.stamp
    
    for joint_name in right_finger_names:
        control_msg.header.frame_id = joint_name
        control_msg.goal_id.id = str(control_msg.header.seq)
        control_msg.goal.command.position = data.data
        control_msg.goal.command.max_effort = 100
        control_msg_tmp = copy.deepcopy(control_msg)
        pubs[joint_name].publish(control_msg_tmp)

def right_thumb_control(data: Float64):
    control_msg = GripperCommandActionGoal()
    control_msg.header.seq += 1
    control_msg.header.stamp = rospy.Time().now()
    control_msg.goal_id.stamp = control_msg.header.stamp
    
    for joint_name in right_thumb_names:
        control_msg.header.frame_id = joint_name
        control_msg.goal_id.id = str(control_msg.header.seq)
        control_msg.goal.command.position = data.data
        control_msg.goal.command.max_effort = 100
        control_msg_tmp = copy.deepcopy(control_msg)
        pubs[joint_name].publish(control_msg_tmp)

if __name__ == '__main__':
    rospy.init_node('fingers_control', anonymous=True)
    rospy.Subscriber('left_fingers_controller/command', Float64,
                     left_fingers_control, queue_size=10)
    rospy.Subscriber('left_thumb_controller/command', Float64,
                     left_thumb_control, queue_size=10)
    rospy.Subscriber('right_fingers_controller/command', Float64,
                     right_fingers_control, queue_size=10)
    rospy.Subscriber('right_thumb_controller/command', Float64,
                     right_thumb_control, queue_size=10)
    rospy.spin()
