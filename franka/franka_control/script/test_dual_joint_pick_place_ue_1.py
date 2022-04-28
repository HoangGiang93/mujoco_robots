#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, ColorRGBA

from giskard_msgs.msg import MoveCmd, JointConstraint
from giskard_msgs.msg import MoveResult, CartesianConstraint, CollisionEntry
from giskard_msgs.msg import MoveAction
from giskard_msgs.msg import MoveGoal

# Brings in the SimpleActionClient
import actionlib
from giskard_msgs.msg import MoveResult

from mujoco_msgs.msg import ObjectStatus, ObjectInfo, ObjectState
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest

import tf

import math
from random import random, uniform, randint

hand_1_pub = [
    rospy.Publisher(
        "dual_panda_arm/panda_1_finger_joint" + str(i) + "_effort_controller/command",
        Float64,
        queue_size=10,
    )
    for i in range(1, 3)
]

# Creates the SimpleActionClient, passing the type of the action
# (MoveAction) to the constructor.
client = actionlib.SimpleActionClient("/giskard_1/command", MoveAction)

cartesian_goal = CartesianConstraint()

cartesian_goal.type = cartesian_goal.POSE_6D

object = ObjectStatus()
types = [ObjectInfo.CUBE, ObjectInfo.SPHERE, ObjectInfo.CYLINDER]
names = ["Cube", "Sphere",  "Cylinder"]

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
    object.info.rgba = ColorRGBA(0.8, 0.1, 0, 1)
    object.info.mesh = "/URoboViz/Assets/StaticMeshes/SM_bowl.SM_bowl"

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
        gen_objects = rospy.ServiceProxy(
            "/unreal/spawn_objects", SpawnObject
        )
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

    object.pose.position.x = uniform(-0.2, 0.2)
    object.pose.position.y = -0.8
    object.pose.position.z = 1.5
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [object]
    try:
        gen_objects = rospy.ServiceProxy(
            "/unreal/spawn_objects", SpawnObject
        )
        gen_objects(objects)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def execute_joint_goal(joint_names, position):
    # Waits until the action server has started up and started
    # listening for goals.
    print("waiting for giskard")
    client.wait_for_server()
    print("connected to giskard")

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
    # if result.error_codes[0] == MoveResult.SUCCESS:
    #     print("giskard returned success")
    # else:
    #     print("something went wrong")


def execute_cartesian_goal(root_link, tip_link, position, orientation):
    # Waits until the action server has started up and started
    # listening for goals.
    print("waiting for giskard")
    client.wait_for_server()
    print("connected to giskard")

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
        print("giskard returned success")
    else:
        print("something went wrong")


def move_to_pre_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            (trans, _) = listener.lookupTransform(hand, object, rospy.Time(0))
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        trans[2] -= 0.1
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.01 and abs(trans[1]) < 0.01:
            return


def move_to_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            (trans, _) = listener.lookupTransform(hand, object, rospy.Time(0))
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.01 and abs(trans[1]) < 0.01 and abs(trans[2]) < 0.01:
            return


def move_to_post_pick(root_link, hand, object):
    while not rospy.is_shutdown():
        try:
            (trans, _) = listener.lookupTransform(hand, object, rospy.Time(0))
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        trans[2] -= 0.2
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        return


def panda_1_open():
    hand_1_pub[0].publish(300)
    hand_1_pub[1].publish(300)
    return


def panda_1_close():
    hand_1_pub[0].publish(0)
    hand_1_pub[1].publish(0)
    return


if __name__ == "__main__":
    rospy.init_node("test_dual_joint_pick_place_1")
    rospy.sleep(1)

    listener = tf.TransformListener()

    rospy.wait_for_service("/unreal/spawn_objects", 1)
    
    
    set_bowl()
    rospy.sleep(1)
    i = 0
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
    while not rospy.is_shutdown():
        panda_1_open()
        move_to_pre_pick("panda_1_link0", "panda_1_hand_tcp", object.info.name)
        move_to_pick("panda_1_link0", "panda_1_hand_tcp", object.info.name)
        panda_1_close()
        move_to_post_pick("panda_1_link0", "panda_1_hand_tcp", object.info.name)
        execute_joint_goal(["panda_1_joint1"], [math.pi / 2])
        panda_1_open()
        set_new_object(i)
        i = i + 1
        execute_joint_goal(["panda_1_joint1"], [-math.pi / 2])
