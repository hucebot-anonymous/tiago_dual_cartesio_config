#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

gripper_cmd_pub = None
gripper_cmd_msg = JointTrajectory()

GRIPPER_MAX = 0.044
GRIPPER_MIN = 0.010

TELEOP_MAX = 0.0
TELEOP_MIN = 1.0

rate_ = 50.0
time_duration = 0.2


def print_info():
    rospy.loginfo(f"    GRIPPER_MAX: {GRIPPER_MAX}")
    rospy.loginfo(f"    GRIPPER_MIN: {GRIPPER_MIN}")
    rospy.loginfo(f"    TELEOP_MAX: {TELEOP_MAX}")
    rospy.loginfo(f"    TELEOP_MIN: {TELEOP_MIN}")


def io_callback(data: Float32):
    a = (GRIPPER_MAX - GRIPPER_MIN) / (TELEOP_MAX - TELEOP_MIN)
    d = data.data
    gripper = a * d * (d - TELEOP_MAX) + GRIPPER_MAX

    for joint_name in gripper_cmd_msg.joint_names:
        gripper_cmd_msg.points[0].positions[
            gripper_cmd_msg.joint_names.index(joint_name)
        ] = gripper
    gripper_cmd_msg.points[0].time_from_start = rospy.Duration(time_duration)

    gripper_cmd_msg.header.stamp = rospy.get_rostime()
    gripper_cmd_pub.publish(gripper_cmd_msg)


def init_cmd_msg(cmd_msg: JointTrajectory, joint_names):
    cmd_msg.joint_names = joint_names
    cmd_msg.points.append(JointTrajectoryPoint())
    cmd_msg.points[0].positions = [0.0] * len(cmd_msg.joint_names)
    cmd_msg.points[0].velocities = [0.0] * len(cmd_msg.joint_names)
    return cmd_msg


if __name__ == "__main__":
    rospy.init_node("ros_control_gripper_bridge", anonymous=True)

    controller_gripper = str()
    if rospy.has_param("~controller_gripper"):
        controller_gripper = rospy.get_param("~controller_gripper")
    else:
        rospy.logerr("controller_gripper private param is mandatory! Exiting.")
        exit()
    rospy.loginfo(f"controller_gripper: {controller_gripper}")

    controller_gripper_joints = []
    if rospy.has_param("~controller_gripper_joints"):
        controller_gripper_joints = rospy.get_param("~controller_gripper_joints")
    else:
        rospy.logerr("controller_gripper_joints private param is mandatory! Exiting.")
        exit()
    rospy.loginfo(f"controller_gripper_joints: {controller_gripper_joints}")

    if rospy.has_param("~GRIPPER_MAX"):
        GRIPPER_MAX = rospy.get_param("~GRIPPER_MAX")
    if rospy.has_param("~GRIPPER_MIN"):
        GRIPPER_MIN = rospy.get_param("~GRIPPER_MIN")
    if rospy.has_param("~TELEOP_MAX"):
        TELEOP_MAX = rospy.get_param("~TELEOP_MAX")
    if rospy.has_param("~TELEOP_MIN"):
        TELEOP_MIN = rospy.get_param("~TELEOP_MIN")

    print_info()

    gripper_cmd_pub = rospy.Publisher(
        controller_gripper + "/command", JointTrajectory, queue_size=1
    )
    gripper_cmd_msg = init_cmd_msg(gripper_cmd_msg, controller_gripper_joints)

    rospy.Subscriber("teleop_gripper", Float32, io_callback)

    rate = rospy.Rate(rate_)
    while not rospy.is_shutdown():
        rate.sleep()
