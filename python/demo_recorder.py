#!/usr/bin/env python3

import numpy as np
import threading
import os

import rospy
import tf2_ros

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped


class DemoRecorder:
    def __init__(self):
        rospy.init_node("demo_recorder")

        # Load params
        if rospy.has_param("~save_folder"):
            self.save_folder = rospy.get_param("~save_folder")
        else:
            self.save_folder = os.environ["HOME"]

        if rospy.has_param("~tags"):
            tags = rospy.get_param("~tags")
        else:
            tags = []

        # Set dump file on node shutdown
        rospy.on_shutdown(self.dump_data)

        # Set up tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Wait for tf listener
        rospy.sleep(1)

        # Init ee pose recorder
        self.actual_left_pose_data = np.zeros((1, 8))
        self.actual_right_pose_data = np.zeros((1, 8))
        self.target_left_pose_data = np.zeros((1, 8))
        self.target_right_pose_data = np.zeros((1, 8))

        # Init joint state recorder
        self.joints_data = np.zeros((1, 26))
        self.sub_joint_state = rospy.Subscriber(
            "/joint_states", JointState, self.cb_joint_state
        )

        # Init left/right gripper recorder
        self.left_gripper_data = np.zeros((1, 3))
        self.right_gripper_data = np.zeros((1, 3))
        self.sub_left_gripper = rospy.Subscriber(
            "/gripper_left_controller/command", JointTrajectory, self.cb_left_gripper
        )
        self.sub_right_gripper = rospy.Subscriber(
            "/gripper_right_controller/command", JointTrajectory, self.cb_right_gripper
        )

        # Init recorder(s) for april tags
        self.tags_data = {}
        for tag in tags:
            self.tags_data[int(tag)] = np.zeros((1, 8))
            rospy.Subscriber(
                f"/inria_orbbec_tags/pose_tag_{tag}",
                PoseStamped,
                self.cb_tag,
                (int(tag)),
            )

    def cb_joint_state(self, msg: JointState):
        """Store joints state data (format: nPoints x [j1, ..., j25, time_stamp])"""
        new_joints = np.zeros((1, 26))
        for i in range(25):
            new_joints[0, i] = msg.position[i]
        new_joints[0, -1] = msg.header.stamp.secs + 10**-9 * msg.header.stamp.nsecs
        self.joints_data = np.append(self.joints_data, new_joints, 0)

    def cb_left_gripper(self, msg: JointTrajectory):
        """Store left gripper data (format: nPoints x [joint_left, joint_right, time_stamp])"""
        new_left_gripper = np.zeros((1, 3))
        new_left_gripper[0, 0] = msg.points[0].positions[
            msg.joint_names.index("gripper_left_left_finger_joint")
        ]
        new_left_gripper[0, 1] = msg.points[0].positions[
            msg.joint_names.index("gripper_left_right_finger_joint")
        ]
        new_left_gripper[0, 2] = msg.header.stamp.secs + 10**-9 * msg.header.stamp.nsecs
        self.left_gripper_data = np.append(self.left_gripper_data, new_left_gripper, 0)

    def cb_right_gripper(self, msg: JointTrajectory):
        """Store right gripper data (format: nPoints x [joint_left, joint_right, time_stamp])"""
        new_right_gripper = np.zeros((1, 3))
        new_right_gripper[0, 0] = msg.points[0].positions[
            msg.joint_names.index("gripper_right_left_finger_joint")
        ]
        new_right_gripper[0, 1] = msg.points[0].positions[
            msg.joint_names.index("gripper_right_right_finger_joint")
        ]
        new_right_gripper[0, 2] = (
            msg.header.stamp.secs + 10**-9 * msg.header.stamp.nsecs
        )
        self.right_gripper_data = np.append(
            self.right_gripper_data, new_right_gripper, 0
        )

    def cb_tag(self, msg: PoseStamped, args: int):
        """Store tags data (format: nTagsx [nPoints x [x, y, z, qx, qy, qz, qw, time_stamp]])"""
        tag_id = args
        new_tag = np.zeros((1, 8))
        new_tag[0, 0] = msg.pose.position.x
        new_tag[0, 1] = msg.pose.position.y
        new_tag[0, 2] = msg.pose.position.z
        new_tag[0, 3] = msg.pose.position.x
        new_tag[0, 4] = msg.pose.orientation.y
        new_tag[0, 5] = msg.pose.orientation.z
        new_tag[0, 6] = msg.pose.orientation.w
        new_tag[0, 7] = msg.header.stamp.secs + 10**-9 * msg.header.stamp.nsecs
        self.tags_data[tag_id] = np.append(self.tags_data[tag_id], new_tag, 0)

    def record(self):
        """Query tf tree and store target and actual ee poses (format: nPoints x [x, y, z, qx, qy, qz, qw, time_stamp])"""
        rospy.loginfo(f"Start recording")
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            try:
                # base-to-left-ee (actual)
                t = self.tf_buffer.lookup_transform(
                    "base_link",
                    "gripper_left_grasping_frame",
                    rospy.Time(),
                    rospy.Duration(1.0),
                )
                new_actual_left_pose = np.zeros((1, 8))
                new_actual_left_pose[0, 1] = t.transform.translation.x
                new_actual_left_pose[0, 2] = t.transform.translation.y
                new_actual_left_pose[0, 3] = t.transform.translation.z
                new_actual_left_pose[0, 4] = t.transform.rotation.x
                new_actual_left_pose[0, 5] = t.transform.rotation.y
                new_actual_left_pose[0, 6] = t.transform.rotation.z
                new_actual_left_pose[0, 7] = t.transform.rotation.w
                new_actual_left_pose[0, 7] = (
                    t.header.stamp.secs + 10**-9 * t.header.stamp.nsecs
                )
                self.actual_left_pose_data = np.append(
                    self.actual_left_pose_data, new_actual_left_pose, 0
                )
            except Exception as err:
                rospy.logwarn(err)

            try:
                # base-to-right-ee (actual)
                t = self.tf_buffer.lookup_transform(
                    "base_link",
                    "gripper_right_grasping_frame",
                    rospy.Time(),
                    rospy.Duration(1.0),
                )
                new_actual_right_pose = np.zeros((1, 8))
                new_actual_right_pose[0, 1] = t.transform.translation.x
                new_actual_right_pose[0, 2] = t.transform.translation.y
                new_actual_right_pose[0, 3] = t.transform.translation.z
                new_actual_right_pose[0, 4] = t.transform.rotation.x
                new_actual_right_pose[0, 5] = t.transform.rotation.y
                new_actual_right_pose[0, 6] = t.transform.rotation.z
                new_actual_right_pose[0, 7] = t.transform.rotation.w
                new_actual_right_pose[0, 7] = (
                    t.header.stamp.secs + 10**-9 * t.header.stamp.nsecs
                )
                self.actual_right_pose_data = np.append(
                    self.actual_right_pose_data, new_actual_right_pose, 0
                )
            except Exception as err:
                rospy.logwarn(err)

            try:
                # base-to-left-ee (target)
                t = self.tf_buffer.lookup_transform(
                    "ci/base_link",
                    "ci/gripper_left_grasping_frame",
                    rospy.Time(),
                    rospy.Duration(1.0),
                )
                new_target_left_pose = np.zeros((1, 8))
                new_target_left_pose[0, 1] = t.transform.translation.x
                new_target_left_pose[0, 2] = t.transform.translation.y
                new_target_left_pose[0, 3] = t.transform.translation.z
                new_target_left_pose[0, 4] = t.transform.rotation.x
                new_target_left_pose[0, 5] = t.transform.rotation.y
                new_target_left_pose[0, 6] = t.transform.rotation.z
                new_target_left_pose[0, 7] = t.transform.rotation.w
                new_target_left_pose[0, 7] = (
                    t.header.stamp.secs + 10**-9 * t.header.stamp.nsecs
                )
                self.target_left_pose_data = np.append(
                    self.target_left_pose_data, new_target_left_pose, 0
                )
            except Exception as err:
                rospy.logwarn(err)

            try:
                # base-to-right-ee (target)
                t = self.tf_buffer.lookup_transform(
                    "ci/base_link",
                    "ci/gripper_right_grasping_frame",
                    rospy.Time(),
                    rospy.Duration(1.0),
                )
                new_target_right_pose = np.zeros((1, 8))
                new_target_right_pose[0, 1] = t.transform.translation.x
                new_target_right_pose[0, 2] = t.transform.translation.y
                new_target_right_pose[0, 3] = t.transform.translation.z
                new_target_right_pose[0, 4] = t.transform.rotation.x
                new_target_right_pose[0, 5] = t.transform.rotation.y
                new_target_right_pose[0, 6] = t.transform.rotation.z
                new_target_right_pose[0, 7] = t.transform.rotation.w
                new_target_right_pose[0, 7] = (
                    t.header.stamp.secs + 10**-9 * t.header.stamp.nsecs
                )
                self.target_right_pose_data = np.append(
                    self.target_right_pose_data, new_target_right_pose, 0
                )
            except Exception as err:
                rospy.logwarn(err)

            rate.sleep()

    def dump_data(self):
        self.sub_joint_state.unregister()
        self.sub_left_gripper.unregister()
        self.sub_right_gripper.unregister()

        rospy.logwarn(
            f"Finished recording, saving recorded data into '{self.save_folder}'..."
        )

        np.save(f"{self.save_folder}/joints_data.npy", self.joints_data)
        np.save(
            f"{self.save_folder}/actual_left_pose_data.npy", self.actual_left_pose_data
        )
        np.save(
            f"{self.save_folder}/actual_right_pose_data.npy",
            self.actual_right_pose_data,
        )
        np.save(
            f"{self.save_folder}/target_left_pose_data.npy", self.target_left_pose_data
        )
        np.save(
            f"{self.save_folder}/target_right_pose_data.npy",
            self.target_right_pose_data,
        )
        np.save(
            f"{self.save_folder}/target_right_pose_data.npy",
            self.target_right_pose_data,
        )
        np.save(f"{self.save_folder}/left_gripper_data.npy", self.left_gripper_data)
        np.save(f"{self.save_folder}/right_gripper_data.npy", self.right_gripper_data)
        for tag in self.tags_data:
            np.save(f"{self.save_folder}/pose_tag_{tag}.npy", self.tags_data[tag])

        for tag in self.tags_data:
            np.savetxt(
                f"{self.save_folder}/pose_tag_{tag}.csv",
                self.tags_data[tag],
                delimiter=",",
            )


if __name__ == "__main__":
    dr = DemoRecorder()
    dr.record()
    dr_thread = threading.Thread(target=dr.record)
    dr_thread.start()

    rospy.spin()
