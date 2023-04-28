#!/usr/bin/env python3

from random import uniform, randint
import math
import tf
from tf2_ros import TransformException

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA

from giskardpy.python_interface import GiskardWrapper

from control_msgs.msg import GripperCommandAction, GripperCommandGoal

# Brings in the SimpleActionClient
import actionlib

from mujoco_msgs.msg import ObjectStatus, ObjectInfo
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest


spawned_object = ObjectStatus()
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
    spawned_object.info.name = "bowl"
    spawned_object.info.type = ObjectInfo.MESH
    spawned_object.info.movable = False
    spawned_object.info.size.x = 1.0
    spawned_object.info.size.y = 1.0
    spawned_object.info.size.z = 1.0
    spawned_object.info.rgba = ColorRGBA(1, 0.65, 0, 1)
    spawned_object.info.mesh = "/Game/Meshes/bowl/bowl.bowl"

    spawned_object.pose.position.x = 0.0
    spawned_object.pose.position.y = 0.0
    spawned_object.pose.position.z = 1.0
    spawned_object.pose.orientation.x = 0.0
    spawned_object.pose.orientation.y = 0.0
    spawned_object.pose.orientation.z = 0.0
    spawned_object.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [spawned_object]

    try:
        gen_objects = rospy.ServiceProxy("/unreal/spawn_objects", SpawnObject)
        gen_objects(objects)
    except rospy.ServiceException as error:
        print(f'Service call failed: {error}')

    # spawned_object.info.mesh = "bowl.xml"
    # try:
    #     gen_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
    #     gen_objects(objects)
    # except rospy.ServiceException as error:
    #     print(f'Service call failed: {error}')


def set_new_object(object_id):
    idx = randint(0, 2)
    spawned_object.info.name = names[idx] + "_1_" + str(object_id)
    spawned_object.info.type = types[idx]
    spawned_object.info.movable = True
    spawned_object.info.size.x = 0.025
    spawned_object.info.size.y = 0.025
    spawned_object.info.size.z = 0.025
    spawned_object.info.rgba = color[randint(0, len(color) - 1)]
    spawned_object.info.inertial.m = 0.01
    if spawned_object.info.type == ObjectInfo.CUBE:
        i_rr = 1/6 * spawned_object.info.inertial.m * \
            spawned_object.info.size.x * spawned_object.info.size.x
        spawned_object.info.inertial.ixx = i_rr
        spawned_object.info.inertial.iyy = i_rr
        spawned_object.info.inertial.izz = i_rr
    elif spawned_object.info.type == ObjectInfo.SPHERE:
        i_rr = 2/5 * spawned_object.info.inertial.m * \
            spawned_object.info.size.x * spawned_object.info.size.x
        spawned_object.info.inertial.ixx = i_rr
        spawned_object.info.inertial.iyy = i_rr
        spawned_object.info.inertial.izz = i_rr
    elif spawned_object.info.type == ObjectInfo.CYLINDER:
        i_rr = 1/12 * spawned_object.info.inertial.m * \
            (3 * spawned_object.info.size.x * spawned_object.info.size.x +
             spawned_object.info.size.z * spawned_object.info.size.z)
        i_zz = 1/2 * spawned_object.info.inertial.m * \
            spawned_object.info.size.x * spawned_object.info.size.x
        spawned_object.info.inertial.ixx = i_rr
        spawned_object.info.inertial.iyy = i_rr
        spawned_object.info.inertial.izz = i_zz

    spawned_object.pose.position.x = uniform(-0.2, 0.2)
    spawned_object.pose.position.y = -0.8
    spawned_object.pose.position.z = 1.2
    spawned_object.pose.orientation.x = 0.0
    spawned_object.pose.orientation.y = 0.0
    spawned_object.pose.orientation.z = 0.0
    spawned_object.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [spawned_object]
    try:
        gen_objects = rospy.ServiceProxy("/unreal/spawn_objects", SpawnObject)
        gen_objects(objects)
    except rospy.ServiceException as error:
        print(f'Service call failed: {error}')

    # try:
    #     gen_objects = rospy.ServiceProxy("/mujoco/spawn_objects", SpawnObject)
    #     gen_objects(objects)
    # except rospy.ServiceException as error:
    #     print(f'Service call failed: {error}')

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
    giskard_wrapper.set_cart_goal(
        goal_pose=goal_pose, tip_link=tip_link, root_link=root_link)
    giskard_wrapper.plan_and_execute(wait=True)


def move_to_pre_pick(root_link, hand, obj):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                hand, obj, rospy.Time(), rospy.Duration(5)
            )
        except TransformException:
            rospy.logwarn(hand + " or " + obj + " not found")
            return

        trans, _ = tf_listener.lookupTransform(
            hand, obj, rospy.Time(0))
        if abs(trans[0]) > 0.5 or abs(trans[2]) > 0.5:
            rospy.logwarn('wtf...')
            continue

        trans[2] -= 0.1
        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.1 and abs(trans[1]) < 0.1:
            return


def move_to_pick(root_link, hand, obj):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                hand, obj, rospy.Time(), rospy.Duration(5)
            )
        except TransformException:
            rospy.logwarn(hand + " or " + obj + " not found")
            return

        trans, _ = tf_listener.lookupTransform(
            hand, obj, rospy.Time(0))

        if abs(trans[0]) > 0.5 or abs(trans[2]) > 0.5:
            rospy.logwarn('wtf...')
            continue

        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        if abs(trans[0]) < 0.1 and abs(trans[1]) < 0.1 and abs(trans[2]) < 0.1:
            return


def move_to_post_pick(root_link, hand, obj):
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform(
                hand, obj, rospy.Time(), rospy.Duration(5)
            )
        except TransformException:
            rospy.logwarn(hand + " or " + obj + " not found")
            return

        trans, _ = tf_listener.lookupTransform(
            hand, obj, rospy.Time(0))

        trans[2] -= 0.2

        execute_cartesian_goal(root_link, hand, trans, [0, 0, 0, 1])
        return


def control_gripper(client: actionlib.SimpleActionClient, open_gripper: bool):
    if open_gripper:
        rospy.loginfo("Open gripper")
    else:
        rospy.loginfo("Close gripper")
    client.wait_for_server()
    gripper_cmd_goal = GripperCommandGoal()
    gripper_cmd_goal.command.position = open_gripper * 0.04
    gripper_cmd_goal.command.max_effort = 1000.0
    client.send_goal(gripper_cmd_goal)


if __name__ == "__main__":
    rospy.init_node("test_dual_joint_pick_place_ue_1")
    rospy.sleep(1)

    tf_listener = tf.TransformListener()

    rospy.wait_for_service("/unreal/spawn_objects", 1)
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
        move_to_pre_pick("panda_1_link0", "panda_1_hand_tcp",
                         spawned_object.info.name)
        move_to_pick("panda_1_link0", "panda_1_hand_tcp",
                     spawned_object.info.name)
        control_gripper(gripper_1_client, False)
        move_to_post_pick("panda_1_link0", "panda_1_hand_tcp",
                          spawned_object.info.name)
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