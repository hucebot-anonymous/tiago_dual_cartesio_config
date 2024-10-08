#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import os

subf = "<subfolder-name>"


actual_left_pose = np.load(os.path.join(subf, "data_actual_left_pose.npy"))[1:, :]
actual_right_pose = np.load(os.path.join(subf, "data_actual_right_pose.npy"))[1:, :]
target_left_pose = np.load(os.path.join(subf, "data_target_left_pose.npy"))[1:, :]
target_right_pose = np.load(os.path.join(subf, "data_target_right_pose.npy"))[1:, :]
right_gripper = np.load(os.path.join(subf, "data_right_gripper.npy"))[1:, :]
left_gripper = np.load(os.path.join(subf, "data_left_gripper.npy"))[1:, :]
pose_tag_0 = np.load(os.path.join(subf, "data_pose_tag_0.npy"))[1:, :]


fig = plt.figure()
fig.suptitle("Left EE", fontsize=16)
plt.plot(
    target_left_pose[:, -1] - target_left_pose[0, -1],
    target_left_pose[:, 0],
    linestyle="--",
    color="tab:red",
    linewidth=2,
    label="target x",
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
    target_left_pose[:, -1] - target_left_pose[0, -1],
    target_left_pose[:, 2],
    color="tab:blue",
    linestyle="--",
    linewidth=2,
    label="target z",
)
plt.legend()
plt.grid()
plt.show()

fig = plt.figure()
fig.suptitle("Right EE", fontsize=16)
plt.plot(
    target_right_pose[:, -1] - target_right_pose[0, -1],
    target_right_pose[:, 0],
    linestyle="--",
    color="tab:red",
    linewidth=2,
    label="target x",
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
    target_right_pose[:, -1] - target_right_pose[0, -1],
    target_right_pose[:, 2],
    color="tab:blue",
    linestyle="--",
    linewidth=2,
    label="target z",
)
plt.legend()
plt.grid()
plt.show()
