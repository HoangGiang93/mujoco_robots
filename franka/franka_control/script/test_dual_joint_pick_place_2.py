#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from giskard_msgs.msg import MoveCmd, JointConstraint
from giskard_msgs.msg import MoveResult, CartesianConstraint, CollisionEntry
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal

# Brings in the SimpleActionClient
import actionlib
from giskard_msgs.msg import MoveResult

from mujoco_msgs.msg import ModelState

import tf

import math
from random import random, uniform, randint

object_pub = rospy.Publisher('panda_arm/create_object', ModelState, queue_size=1)
hand_2_pub = [rospy.Publisher('panda_arm/panda_2_finger_joint' + str(i) + '_effort_controller/command', Float64, queue_size=10) for i in range(1,3)]

# Creates the SimpleActionClient, passing the type of the action
# (MoveAction) to the constructor.
client = actionlib.SimpleActionClient("/giskard_2/command", MoveAction)

cartesian_goal = CartesianConstraint()
 
cartesian_goal.type = cartesian_goal.POSE_6D

object = ModelState()
types = [ModelState.CUBE, ModelState.SPHERE, ModelState.CYLINDER]

def set_new_object(i):
    object.name = 'object_2_' + str(i)
    object.pose.position.x = uniform(-0.2, 0.2)
    object.pose.position.y = 0.8 
    object.pose.position.z = 1.5
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0

    object.type = types[randint(0, 2)]
    object.scale.x = 0.025
    object.scale.y = 0.025
    object.scale.z = 0.025
    object.color.r = random()
    object.color.g = random()
    object.color.b = random()
    object.color.a = 1.0
    object_pub.publish(object)

def execute_joint_goal(joint_names, position):
    # Waits until the action server has started up and started
    # listening for goals.
    print('waiting for giskard')
    client.wait_for_server()
    print('connected to giskard')
    
    # Creates a goal to send to the action server.
    action_goal = MoveGoal()

    action_goal.type = MoveGoal.PLAN_AND_EXECUTE

    joint_goal = JointConstraint()

    joint_goal.type = JointConstraint.JOINT

    # this can be any subset of the robots joints
    # joint_goal.goal_state is a normal sensor_msgs/JointState
    joint_goal.goal_state.name = joint_names

    joint_goal.goal_state.position = position

    goal = MoveCmd()
    # allow all collision
    ce = CollisionEntry()
    ce.robot_links = [CollisionEntry.ALL]
    ce.body_b = CollisionEntry.ALL
    ce.link_bs = [CollisionEntry.ALL]
    ce.type = CollisionEntry.ALLOW_COLLISION
    goal.collisions = [ce]

    goal.joint_constraints = [joint_goal]
    action_goal.cmd_seq = [goal]
 
    # Sends the goal to the action server.
    client.send_goal(action_goal)
 
    # Waits for the server to finish performing the action.
    client.wait_for_result()
 
    result = client.get_result()  # type: MoveResult
    if result.error_codes[0] == MoveResult.SUCCESS:
        print('giskard returned success')
    else:
        print('something went wrong')

def execute_cartesian_goal(root_link, tip_link, position, orientation):
    # Waits until the action server has started up and started
    # listening for goals.
    print('waiting for giskard')
    client.wait_for_server()
    print('connected to giskard')
    
    # Creates a goal to send to the action server.
    action_goal = MoveGoal()

    action_goal.type = MoveGoal.PLAN_AND_EXECUTE
 
    # specify the kinematic chain
    cartesian_goal.root_link = root_link
    cartesian_goal.tip_link = tip_link
 
    cartesian_goal.goal.header.frame_id = tip_link
    cartesian_goal.goal.pose.position.x = position[0]
    cartesian_goal.goal.pose.position.y = position[1]
    cartesian_goal.goal.pose.position.z = position[2]
    cartesian_goal.goal.pose.orientation.x = orientation[0]
    cartesian_goal.goal.pose.orientation.y = orientation[1]
    cartesian_goal.goal.pose.orientation.z = orientation[2]
    cartesian_goal.goal.pose.orientation.w = orientation[3]

    goal = MoveCmd()
    # allow all collision
    ce = CollisionEntry()
    ce.robot_links = [CollisionEntry.ALL]
    ce.body_b = CollisionEntry.ALL
    ce.link_bs = [CollisionEntry.ALL]
    ce.type = CollisionEntry.ALLOW_COLLISION
    goal.collisions = [ce]
 
    goal.cartesian_constraints = [cartesian_goal]
 
    action_goal.cmd_seq = [goal]
 
    # Sends the goal to the action server.
    client.send_goal(action_goal)
 
    # Waits for the server to finish performing the action.
    client.wait_for_result()
 
    result = client.get_result()  # type: MoveResult
    if result.error_codes[0] == MoveResult.SUCCESS:
        print('giskard returned success')
    else:
        print('something went wrong')

def move_to_pre_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            (trans, _) = listener.lookupTransform(hand, object, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        trans[2] -= 0.1
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.01 and abs(trans[1]) < 0.01:
            return

def move_to_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            (trans, _) = listener.lookupTransform(hand, object, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.01 and abs(trans[1]) < 0.01 and abs(trans[2]) < 0.01:
            return

def move_to_post_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            (trans, _) = listener.lookupTransform(hand, object, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        trans[2] -= 0.2
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        return

def panda_2_open():
    hand_2_pub[0].publish(300)
    hand_2_pub[1].publish(300)
    return

def panda_2_close():
    hand_2_pub[0].publish(0)
    hand_2_pub[1].publish(0)
    return

if __name__ == '__main__':
    rospy.init_node('test_dual_joint_pick_place_2')
    rospy.sleep(1)

    listener = tf.TransformListener()

    i = 0
    set_new_object(i)
    i = i + 1
    execute_joint_goal(["panda_2_joint1", "panda_2_joint2", "panda_2_joint3", "panda_2_joint4", "panda_2_joint5", "panda_2_joint6", "panda_2_joint7"], [math.pi/2, 0.37, 0.0, -2.22, 0, 2.56, 0.8])
    while not rospy.is_shutdown():
        execute_joint_goal(["panda_1_joint1"], [-math.pi/2])
        
        panda_2_open()
        move_to_pre_pick('panda_2_link0', 'panda_2_hand_tcp', object.name)
        move_to_pick('panda_2_link0', 'panda_2_hand_tcp', object.name)
        panda_2_close()
        move_to_post_pick('panda_2_link0', 'panda_2_hand_tcp', object.name)
        execute_joint_goal(["panda_2_joint1"], [-math.pi/2])
        panda_2_open()
        set_new_object(i)
        i = i + 1
        execute_joint_goal(["panda_2_joint1"], [math.pi/2])


    