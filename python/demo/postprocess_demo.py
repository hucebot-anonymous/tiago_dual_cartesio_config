#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import os

joint_names = [
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
    "gripper_left_left_finger_joint",
    "gripper_left_right_finger_joint",
    "gripper_right_left_finger_joint",
    "gripper_right_right_finger_joint",
    "head_1_joint",
    "head_2_joint",
    "torso_lift_joint",
    "wheel_front_left_joint",
    "wheel_front_right_joint",
    "wheel_rear_left_joint",
    "wheel_rear_right_joint",
]


subf = "dummy_log"

joints = np.load(os.path.join(subf, "data_joints.npy"))[1:, :]
print("joints.shape", joints.shape)

actual_left_pose = np.load(os.path.join(subf, "data_actual_left_pose.npy"))[1:, :]
print("actual_left_pose.shape", actual_left_pose.shape)
actual_right_pose = np.load(os.path.join(subf, "data_actual_right_pose.npy"))[1:, :]
print("actual_right_pose.shape", actual_right_pose.shape)

target_left_pose = np.load(os.path.join(subf, "data_target_left_pose.npy"))[1:, :]
print("target_left_pose.shape", target_left_pose.shape)
target_right_pose = np.load(os.path.join(subf, "data_target_right_pose.npy"))[1:, :]
print("target_right_pose.shape", target_right_pose.shape)

pose_tag_0 = np.load(os.path.join(subf, "data_pose_tag_0.npy"))[1:, :]
print("pose_tag_0.shape", pose_tag_0.shape)


fig = plt.figure()
fig.suptitle("Left EE", fontsize=16)
plt.plot(
    actual_left_pose[:, -1] - actual_left_pose[0, -1],
    actual_left_pose[:, 0],
    color="tab:red",
    linewidth=2,
    label="true x",
)
plt.plot(
    target_left_pose[:, -1] - target_left_pose[0, -1],
    target_left_pose[:, 0],
    linestyle="--",
    color="tab:red",
    linewidth=2,
    label="target x",
)
plt.plot(
    actual_left_pose[:, -1] - actual_left_pose[0, -1],
    actual_left_pose[:, 1],
    color="tab:green",
    linewidth=2,
    label="true y",
)
plt.plot(
    target_left_pose[:, -1] - target_left_pose[0, -1],
    target_left_pose[:, 1],
    linestyle="--",
    color="tab:green",
    linewidth=2,
    label="target y",
)
plt.plot(
    actual_left_pose[:, -1] - actual_left_pose[0, -1],
    actual_left_pose[:, 2],
    color="tab:blue",
    linewidth=2,
    label="true z",
)
plt.plot(
    target_left_pose[:, -1] - target_left_pose[0, -1],
    target_left_pose[:, 2],
    color="tab:blue",
    linestyle="--",
    linewidth=2,
    label="target z",
)
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.show()

fig = plt.figure()
fig.suptitle("Right EE", fontsize=16)
plt.plot(
    actual_right_pose[:, -1] - actual_right_pose[0, -1],
    actual_right_pose[:, 0],
    color="tab:red",
    linewidth=2,
    label="true x",
)
plt.plot(
    target_right_pose[:, -1] - target_right_pose[0, -1],
    target_right_pose[:, 0],
    linestyle="--",
    color="tab:red",
    linewidth=2,
    label="target x",
)
plt.plot(
    actual_right_pose[:, -1] - actual_right_pose[0, -1],
    actual_right_pose[:, 1],
    color="tab:green",
    linewidth=2,
    label="true y",
)
plt.plot(
    target_right_pose[:, -1] - target_right_pose[0, -1],
    target_right_pose[:, 1],
    linestyle="--",
    color="tab:green",
    linewidth=2,
    label="target y",
)
plt.plot(
    actual_right_pose[:, -1] - actual_right_pose[0, -1],
    actual_right_pose[:, 2],
    color="tab:blue",
    linewidth=2,
    label="true z",
)
plt.plot(
    target_right_pose[:, -1] - target_right_pose[0, -1],
    target_right_pose[:, 2],
    color="tab:blue",
    linestyle="--",
    linewidth=2,
    label="target z",
)
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.show()

fig = plt.figure()
fig.suptitle("Pose Tag 0", fontsize=16)
plt.plot(
    pose_tag_0[:, -1] - pose_tag_0[0, -1],
    pose_tag_0[:, 0],
    color="tab:red",
    linewidth=2,
    label="x",
)
plt.plot(
    pose_tag_0[:, -1] - pose_tag_0[0, -1],
    pose_tag_0[:, 1],
    color="tab:green",
    linewidth=2,
    label="y",
)
plt.plot(
    pose_tag_0[:, -1] - pose_tag_0[0, -1],
    pose_tag_0[:, 2],
    color="tab:blue",
    linewidth=2,
    label="z",
)
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.show()


fig = plt.figure()
fig.suptitle("Left Gripper", fontsize=16)
plt.plot(
    joints[:, -1] - joints[0, -1],
    joints[:, 14],
    color="tab:red",
    linewidth=2,
    label="left finger joint",
)
plt.plot(
    joints[:, -1] - joints[0, -1],
    joints[:, 15],
    color="tab:green",
    linewidth=2,
    label="right finger joint",
)
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.show()

fig = plt.figure()
fig.suptitle("Right Gripper", fontsize=16)
plt.plot(
    joints[:, -1] - joints[0, -1],
    joints[:, 16],
    color="tab:red",
    linewidth=2,
    label="left finger joint",
)
plt.plot(
    joints[:, -1] - joints[0, -1],
    joints[:, 17],
    color="tab:green",
    linewidth=2,
    label="right finger joint",
)
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.legend()
plt.grid()
plt.show()

fig = plt.figure()
fig.suptitle("Left arm joints", fontsize=16)
plt.xlabel("Time [s]")
start_idx = 0
for i in range(7):
    plt.plot(
        joints[:, -1] - joints[0, -1],
        joints[:, start_idx + i],
        linewidth=2,
        label=joint_names[start_idx + i],
    )
plt.xlabel("Time [s]")
plt.ylabel("Angle [rad]")
plt.legend()
plt.grid()
plt.show()

fig = plt.figure()
fig.suptitle("Right arm joints", fontsize=16)
plt.xlabel("Time [s]")
start_idx = 7
for i in range(7):
    plt.plot(
        joints[:, -1] - joints[0, -1],
        joints[:, start_idx + i],
        linewidth=2,
        label=joint_names[start_idx + i],
    )
plt.xlabel("Time [s]")
plt.ylabel("Angle [rad]")
plt.legend()
plt.grid()
plt.show()
