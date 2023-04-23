#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, ColorRGBA

from giskardpy.python_interface import GiskardWrapper

from control_msgs.msg import GripperCommandAction, GripperCommandGoal

# Brings in the SimpleActionClient
import actionlib
from giskard_msgs.msg import MoveResult

from mujoco_msgs.msg import ObjectStatus, ObjectInfo, ObjectState
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest

from geometry_msgs.msg import PoseStamped

import tf
import tf2_ros
from tf2_ros import TransformException

import math
from random import random, uniform, randint

object = ObjectStatus()
types = [ObjectInfo.CUBE, ObjectInfo.SPHERE, ObjectInfo.CYLINDER]
names = ["Cube", "Sphere", "Cylinder"]

color = [
    ColorRGBA(0, 0, 1, 1),
    ColorRGBA(0, 1, 1, 1),
    ColorRGBA(0, 1, 0, 1),
    ColorRGBA(1, 0, 0.5, 1),
    ColorRGBA(0.5, 0, 1, 1),
    ColorRGBA(1, 0, 0, 1),
    ColorRGBA(1, 1, 0, 1),
]

def set_bowl():
    object.info.name = "bowl"
    object.info.type = ObjectInfo.MESH
    object.info.movable = False
    object.info.size.x = 1.0
    object.info.size.y = 1.0
    object.info.size.z = 1.0
    object.info.rgba = ColorRGBA(1, 0.65, 0, 1)
    object.info.mesh = "bowl.xml"
    
    object.pose.position.x = 0.0
    object.pose.position.y = 0.0
    object.pose.position.z = 1.0
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [object]

    try:
        gen_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
        gen_objects(objects)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def set_new_object(i):
    idx = randint(0, 2)
    object.info.name = names[idx] + "_1_" + str(i)
    object.info.type = types[idx]
    object.info.movable = True
    object.info.size.x = 0.025
    object.info.size.y = 0.025
    object.info.size.z = 0.025
    object.info.rgba = color[randint(0, len(color) - 1)]
    object.info.inertial.m = 0.01
    if object.info.type == ObjectInfo.CUBE:
        i_rr = 1/6 * object.info.inertial.m * object.info.size.x * object.info.size.x
        object.info.inertial.ixx = i_rr
        object.info.inertial.iyy = i_rr
        object.info.inertial.izz = i_rr
    elif object.info.type == ObjectInfo.SPHERE:
        i_rr = 2/5 * object.info.inertial.m * object.info.size.x * object.info.size.x
        object.info.inertial.ixx = i_rr
        object.info.inertial.iyy = i_rr
        object.info.inertial.izz = i_rr
    elif object.info.type == ObjectInfo.CYLINDER:
        i_rr = 1/12 * object.info.inertial.m * (3 * object.info.size.x * object.info.size.x + object.info.size.z * object.info.size.z)
        i_zz = 1/2 * object.info.inertial.m * object.info.size.x * object.info.size.x
        object.info.inertial.ixx = i_rr
        object.info.inertial.iyy = i_rr
        object.info.inertial.izz = i_zz

    object.pose.position.x = uniform(-0.2, 0.2)
    object.pose.position.y = -0.8
    object.pose.position.z = 1.2
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [object]
    try:
        gen_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
        gen_objects(objects)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def execute_joint_goal(joint_names, joint_positions):
    giskard_wrapper.set_joint_goal(dict(zip(joint_names, joint_positions)))
    giskard_wrapper.plan_and_execute(wait=True)

def execute_cartesian_goal(root_link, tip_link, position, orientation):
    rospy.loginfo('Move ' + tip_link + ' to ' + str(position))
    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = tip_link
    goal_pose.pose.position.x = position[0]
    goal_pose.pose.position.y = position[1]
    goal_pose.pose.position.z = position[2]
    goal_pose.pose.orientation.x = orientation[0]
    goal_pose.pose.orientation.y = orientation[1]
    goal_pose.pose.orientation.z = orientation[2]
    goal_pose.pose.orientation.w = orientation[3]
    giskard_wrapper.set_cart_goal(goal_pose=goal_pose, tip_link=tip_link, root_link=root_link)
    giskard_wrapper.plan_and_execute(wait=True)

def move_to_pre_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                hand, object, rospy.Time(), rospy.Duration(5)
            )
        except TransformException as e:
            rospy.logwarn(hand + " or " + object + " not found")
            return
            
        trans, _ = tf_listener.lookupTransform(hand, object, rospy.Time(0))
        if abs(trans[0]) > 0.5 or abs(trans[2]) > 0.5:
            rospy.logwarn('wtf...')
            continue

        trans[2] -= 0.1
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.1 and abs(trans[1]) < 0.1:
            return


def move_to_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                hand, object, rospy.Time(), rospy.Duration(5)
            )
        except TransformException as e:
            rospy.logwarn(hand + " or " + object + " not found")
            return
            
        trans, _ = tf_listener.lookupTransform(hand, object, rospy.Time(0))
        
        if abs(trans[0]) > 0.5 or abs(trans[2]) > 0.5:
            rospy.logwarn('wtf...')
            continue
        
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.1 and abs(trans[1]) < 0.1 and abs(trans[2]) < 0.1:
            return


def move_to_post_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                hand, object, rospy.Time(), rospy.Duration(5)
            )
        except TransformException as e:
            rospy.logwarn(hand + " or " + object + " not found")
            return
            
        trans, _ = tf_listener.lookupTransform(hand, object, rospy.Time(0))
        
        trans[2] -= 0.2

        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        return


def control_gripper(client: actionlib.SimpleActionClient, open: bool):
    if open:
        rospy.loginfo("Open gripper")
    else:
        rospy.loginfo("Close gripper")
    client.wait_for_server()
    gripper_cmd_goal = GripperCommandGoal()
    gripper_cmd_goal.command.position = open * 0.04
    gripper_cmd_goal.command.max_effort = 1000.0
    client.send_goal(gripper_cmd_goal)
    return None


if __name__ == "__main__":
    rospy.init_node("test_dual_joint_pick_place_1")
    rospy.sleep(1)

    tf_listener = tf.TransformListener()

    rospy.wait_for_service("/mujoco/spawn_objects", 1)

    gripper_1_client = actionlib.SimpleActionClient(
        "/dual_panda/panda_1_gripper_controller/gripper_cmd", GripperCommandAction
    )

    set_bowl()

    giskard_wrapper = GiskardWrapper(node_name='giskard_1')
    execute_joint_goal(
        [
            "panda_1_joint1",
            "panda_1_joint2",
            "panda_1_joint3",
            "panda_1_joint4",
            "panda_1_joint5",
            "panda_1_joint6",
            "panda_1_joint7",
        ],
        [-math.pi / 2, 0.37, 0.0, -2.22, 0, 2.56, 0.8],
    )

    i = 0
    set_new_object(i)
    i = i + 1
    
    while not rospy.is_shutdown():
        control_gripper(gripper_1_client, True)
        move_to_pre_pick("panda_1_link0", "panda_1_hand_tcp", object.info.name)
        move_to_pick("panda_1_link0", "panda_1_hand_tcp", object.info.name)
        control_gripper(gripper_1_client, False)
        move_to_post_pick("panda_1_link0", "panda_1_hand_tcp", object.info.name)
        execute_joint_goal(["panda_1_joint1"], [math.pi / 2])
        control_gripper(gripper_1_client, True)
        set_new_object(i)
        i = i + 1
        execute_joint_goal(
        [
            "panda_1_joint1",
            "panda_1_joint2",
            "panda_1_joint3",
            "panda_1_joint4",
            "panda_1_joint5",
            "panda_1_joint6",
            "panda_1_joint7",
        ],
        [-math.pi / 2, 0.37, 0.0, -2.22, 0, 2.56, 0.8],
    )
