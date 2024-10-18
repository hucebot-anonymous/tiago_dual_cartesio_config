#!/usr/bin/env python3
from colorama import Fore, Style
from datetime import datetime
import glob
import numpy as np
import os
from select import select
import sys
import threading

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty

import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped


class DemoRecorder:
    def __init__(self, folder_path=os.environ["HOME"], cameras=[], tags=[]):

        # Init member variables
        self.folder_path = folder_path
        self.cameras = cameras
        self.tags = tags
        self.demo_idx = 0
        self.flg_record = False

        # Set up tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Select new initial demo index
        subfolders = [f.path for f in os.scandir(self.folder_path) if f.is_dir()]
        for subf in subfolders:
            try:
                i = int(
                    subf[
                        subf.startswith(os.path.join(self.folder_path, ""))
                        and len(os.path.join(self.folder_path, "")) :
                    ]
                )
                if i >= self.demo_idx:
                    self.demo_idx = i + 1
            except:
                pass
        self.save_folder = os.path.join(self.folder_path, str(self.demo_idx))
        os.mkdir(self.save_folder)

        self.init_log_variables()
        self.init_subscribers()

        # Wait for tf listener
        rospy.sleep(1)

        print("Demo recorder initialized")
        print(f"Demo index = {self.demo_idx}")

    def init_log_variables(self):
        # Init joint state recorder
        self.data_joints = np.zeros((1, 26))

        # Init recorder(s) for pose tags
        self.data_tags = {}
        for camera in self.cameras:
            self.data_tags[camera] = {}
            for tag in self.tags:
                self.data_tags[camera][int(tag)] = np.zeros((1, 8))

        # Init ee pose recorder
        self.data_measured_left_pose = np.zeros((1, 8))
        self.data_cartesio_left_pose = np.zeros((1, 8))
        self.data_teleop_left_pose = np.zeros((1, 8))
        self.data_teleop_right_pose = np.zeros((1, 8))
        self.data_cartesio_right_pose = np.zeros((1, 8))
        self.data_measured_right_pose = np.zeros((1, 8))

        # Init gripper commands recorder
        self.data_left_gripper_cmd = np.zeros((1, 2))
        self.data_right_gripper_cmd = np.zeros((1, 2))

    def init_subscribers(self):
        # Init joint state recorder
        self.sub_joint_state = rospy.Subscriber(
            "/joint_states", JointState, self.cb_joint_state
        )

        # Init recorder(s) for pose tags
        for camera in self.cameras:
            for tag in self.tags:
                rospy.Subscriber(
                    f"/inria_orbbec_tags/pose_tag_{tag}",
                    PoseStamped,
                    self.cb_tag,
                    (camera, int(tag)),
                )

        # Init ee pose recorder
        self.sub_left_teleop = rospy.Subscriber(
            "/dxl_input/pos_left", PoseStamped, self.cb_left_teleop
        )

        self.sub_right_teleop = rospy.Subscriber(
            "/dxl_input/pos_right", PoseStamped, self.cb_right_teleop
        )

        # Init gripper commands recorder
        self.sub_left_gripper_cmd = rospy.Subscriber(
            "/dxl_input/gripper_left", Float32, self.cb_left_gripper
        )

        self.sub_right_gripper_cmd = rospy.Subscriber(
            "/dxl_input/gripper_right", Float32, self.cb_right_gripper
        )

    def cb_joint_state(self, msg: JointState):
        """Store joints state data (format: nPoints x [j1, ..., j25, time_stamp])"""
        if self.flg_record:
            new_joints = np.zeros((1, 26))
            for i in range(25):
                new_joints[0, i] = msg.position[i]
            new_joints[0, -1] = int(datetime.now().timestamp() * 1000)  # [ms]
            self.data_joints = np.append(self.data_joints, new_joints, 0)

    def cb_tag(self, msg: PoseStamped, args):
        """Store tags data (format: nTagsx [nPoints x [x, y, z, qx, qy, qz, qw, time_stamp]])"""
        if self.flg_record:
            camera_name = args[0]
            tag_id = args[1]
            new_tag = np.zeros((1, 8))
            new_tag[0, 0] = msg.pose.position.x
            new_tag[0, 1] = msg.pose.position.y
            new_tag[0, 2] = msg.pose.position.z
            new_tag[0, 3] = msg.pose.orientation.x
            new_tag[0, 4] = msg.pose.orientation.y
            new_tag[0, 5] = msg.pose.orientation.z
            new_tag[0, 6] = msg.pose.orientation.w
            new_tag[0, 7] = int(
                msg.header.stamp.secs * 1000 + msg.header.stamp.nsecs * 10**-6
            )  # [ms]
            self.data_tags[camera_name][tag_id] = np.append(
                self.data_tags[camera_name][tag_id], new_tag, 0
            )

    def cb_left_teleop(self, msg: PoseStamped):
        """Store teleop pose, cartesio and measured ee poses (format: nPoints x [x, y, z, qx, qy, qz, qw, time_stamp])"""
        if self.flg_record:
            try:
                timestamp = int(datetime.now().timestamp() * 1000)

                # base-to-left-ee (teleop)
                new_teleop_left_pose = np.zeros((1, 8))
                new_teleop_left_pose[0, 0] = msg.pose.position.x
                new_teleop_left_pose[0, 1] = msg.pose.position.y
                new_teleop_left_pose[0, 2] = msg.pose.position.z
                new_teleop_left_pose[0, 3] = msg.pose.orientation.x
                new_teleop_left_pose[0, 4] = msg.pose.orientation.y
                new_teleop_left_pose[0, 5] = msg.pose.orientation.z
                new_teleop_left_pose[0, 6] = msg.pose.orientation.w
                new_teleop_left_pose[0, 7] = timestamp
                self.data_teleop_left_pose = np.append(
                    self.data_teleop_left_pose, new_teleop_left_pose, 0
                )

                # base-to-left-ee (measured)
                t = self.tf_buffer.lookup_transform(
                    "base_link",
                    "gripper_left_grasping_frame",
                    rospy.Time(),
                )
                new_measured_left_pose = np.zeros((1, 8))
                new_measured_left_pose[0, 0] = t.transform.translation.x
                new_measured_left_pose[0, 1] = t.transform.translation.y
                new_measured_left_pose[0, 2] = t.transform.translation.z
                new_measured_left_pose[0, 3] = t.transform.rotation.x
                new_measured_left_pose[0, 4] = t.transform.rotation.y
                new_measured_left_pose[0, 5] = t.transform.rotation.z
                new_measured_left_pose[0, 6] = t.transform.rotation.w
                new_measured_left_pose[0, 7] = timestamp
                self.data_measured_left_pose = np.append(
                    self.data_measured_left_pose, new_measured_left_pose, 0
                )

                # base-to-left-ee (cartesio)
                t = self.tf_buffer.lookup_transform(
                    "ci/base_link",
                    "ci/gripper_left_grasping_frame",
                    rospy.Time(),
                )
                new_cartesio_left_pose = np.zeros((1, 8))
                new_cartesio_left_pose[0, 0] = t.transform.translation.x
                new_cartesio_left_pose[0, 1] = t.transform.translation.y
                new_cartesio_left_pose[0, 2] = t.transform.translation.z
                new_cartesio_left_pose[0, 3] = t.transform.rotation.x
                new_cartesio_left_pose[0, 4] = t.transform.rotation.y
                new_cartesio_left_pose[0, 5] = t.transform.rotation.z
                new_cartesio_left_pose[0, 6] = t.transform.rotation.w
                new_cartesio_left_pose[0, 7] = timestamp
                self.data_cartesio_left_pose = np.append(
                    self.data_cartesio_left_pose, new_cartesio_left_pose, 0
                )
            except Exception as err:
                print(Fore.RED + err)
                print(Style.RESET_ALL)

    def cb_right_teleop(self, msg: PoseStamped):
        """Store teleop pose, cartesio and measured ee poses (format: nPoints x [x, y, z, qx, qy, qz, qw, time_stamp])"""
        if self.flg_record:
            try:
                timestamp = int(datetime.now().timestamp() * 1000)

                # base-to-right-ee (teleop)
                new_teleop_right_pose = np.zeros((1, 8))
                new_teleop_right_pose[0, 0] = msg.pose.position.x
                new_teleop_right_pose[0, 1] = msg.pose.position.y
                new_teleop_right_pose[0, 2] = msg.pose.position.z
                new_teleop_right_pose[0, 3] = msg.pose.orientation.x
                new_teleop_right_pose[0, 4] = msg.pose.orientation.y
                new_teleop_right_pose[0, 5] = msg.pose.orientation.z
                new_teleop_right_pose[0, 6] = msg.pose.orientation.w
                new_teleop_right_pose[0, 7] = timestamp
                self.data_teleop_right_pose = np.append(
                    self.data_teleop_right_pose, new_teleop_right_pose, 0
                )

                # base-to-right-ee (measured)
                t = self.tf_buffer.lookup_transform(
                    "base_link",
                    "gripper_right_grasping_frame",
                    rospy.Time(),
                )
                new_measured_right_pose = np.zeros((1, 8))
                new_measured_right_pose[0, 0] = t.transform.translation.x
                new_measured_right_pose[0, 1] = t.transform.translation.y
                new_measured_right_pose[0, 2] = t.transform.translation.z
                new_measured_right_pose[0, 3] = t.transform.rotation.x
                new_measured_right_pose[0, 4] = t.transform.rotation.y
                new_measured_right_pose[0, 5] = t.transform.rotation.z
                new_measured_right_pose[0, 6] = t.transform.rotation.w
                new_measured_right_pose[0, 7] = timestamp
                self.data_measured_right_pose = np.append(
                    self.data_measured_right_pose, new_measured_right_pose, 0
                )

                # base-to-right-ee (cartesio)
                t = self.tf_buffer.lookup_transform(
                    "ci/base_link",
                    "ci/gripper_right_grasping_frame",
                    rospy.Time(),
                )
                new_cartesio_right_pose = np.zeros((1, 8))
                new_cartesio_right_pose[0, 0] = t.transform.translation.x
                new_cartesio_right_pose[0, 1] = t.transform.translation.y
                new_cartesio_right_pose[0, 2] = t.transform.translation.z
                new_cartesio_right_pose[0, 3] = t.transform.rotation.x
                new_cartesio_right_pose[0, 4] = t.transform.rotation.y
                new_cartesio_right_pose[0, 5] = t.transform.rotation.z
                new_cartesio_right_pose[0, 6] = t.transform.rotation.w
                new_cartesio_right_pose[0, 7] = timestamp
                self.data_cartesio_right_pose = np.append(
                    self.data_cartesio_right_pose, new_cartesio_right_pose, 0
                )
            except Exception as err:
                print(Fore.RED + err)
                print(Style.RESET_ALL)

    def cb_left_gripper(self, msg: Float32):
        if self.flg_record:
            timestamp = int(datetime.now().timestamp() * 1000)
            new_left_gripper_cmd = np.zeros((1, 2))
            new_left_gripper_cmd[0, 0] = msg.data
            new_left_gripper_cmd[0, 1] = timestamp
            self.data_left_gripper_cmd = np.append(
                self.data_left_gripper_cmd, new_left_gripper_cmd, 0
            )

    def cb_right_gripper(self, msg: Float32):
        if self.flg_record:
            timestamp = int(datetime.now().timestamp() * 1000)
            new_right_gripper_cmd = np.zeros((1, 2))
            new_right_gripper_cmd[0, 0] = msg.data
            new_right_gripper_cmd[0, 1] = timestamp
            self.data_right_gripper_cmd = np.append(
                self.data_right_gripper_cmd, new_right_gripper_cmd, 0
            )

    def set_state(self, state: bool):
        self.flg_record = state
        if state:
            print(f"Start logging demo {dr.get_demo_idx()}")
        else:
            print(f"Stop logging demo {dr.get_demo_idx()}")

    def get_state(self):
        return self.flg_record

    def get_demo_idx(self):
        return self.demo_idx

    def new_demo(self):
        try:
            self.demo_idx += 1
            self.save_folder = os.path.join(self.folder_path, str(self.demo_idx))
            os.mkdir(self.save_folder)
            print(f"New demo index = {self.demo_idx}")
        except Exception as err:
            print(Fore.RED + err)
            print(Style.RESET_ALL)

    def cancel_log(self):
        if self.flg_record:
            print(Fore.YELLOW + "Cancel not allowed while log is enabled")
            print(Style.RESET_ALL)
        else:
            print(f"Removing all npy files in {self.save_folder}/")
            files = glob.glob(f"{self.save_folder}/*.npy")
            for f in files:
                os.remove(f)

    def save_data(self):
        if self.flg_record:
            print(Fore.YELLOW + "Saving not allowed while log is enabled")
            print(Style.RESET_ALL)
        else:
            print(Fore.GREEN + f"Saving demo '{self.demo_idx} ...")

            np.save(f"{self.save_folder}/data_joints.npy", self.data_joints)
            np.save(
                f"{self.save_folder}/data_teleop_left_pose.npy",
                self.data_teleop_left_pose,
            )
            np.save(
                f"{self.save_folder}/data_teleop_right_pose.npy",
                self.data_teleop_right_pose,
            )
            np.save(
                f"{self.save_folder}/data_measured_left_pose.npy",
                self.data_measured_left_pose,
            )
            np.save(
                f"{self.save_folder}/data_measured_right_pose.npy",
                self.data_measured_right_pose,
            )
            np.save(
                f"{self.save_folder}/data_cartesio_left_pose.npy",
                self.data_cartesio_left_pose,
            )
            np.save(
                f"{self.save_folder}/data_cartesio_right_pose.npy",
                self.data_cartesio_right_pose,
            )
            np.save(
                f"{self.save_folder}/data_cartesio_right_pose.npy",
                self.data_cartesio_right_pose,
            )
            np.save(
                f"{self.save_folder}/data_left_gripper_cmd.npy",
                self.data_left_gripper_cmd,
            )
            np.save(
                f"{self.save_folder}/data_right_gripper_cmd.npy",
                self.data_right_gripper_cmd,
            )
            for camera in self.cameras:
                for tag in self.tags:
                    np.save(
                        f"{self.save_folder}/data_pose_tag_{tag}_from_{camera}.npy",
                        self.data_tags[camera][int(tag)],
                    )

            self.init_log_variables()

            print(Fore.GREEN + "Saving complete")
            print(Style.RESET_ALL)
            self.new_demo()


def getKey(settings, timeout):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def terminal_menu(dr, menu, key_timeout=0.5):
    # TODO: add publish to orbbec cmd topic
    settings = saveTerminalSettings()
    try:
        print(menu_str)
        while 1:
            key = getKey(settings, key_timeout)
            if key == " ":
                dr.set_state(not dr.get_state())
            if key == "i":
                print(f"Current demo index = {dr.get_demo_idx()}")
            if key == "s":
                dr.save_data()
                print(menu_str)
            if key == "c":
                dr.cancel_log()
                print(menu_str)
            if key == "\x03":
                break

    except Exception as e:
        print(e)

    finally:
        restoreTerminalSettings(settings)


menu_str = """
            DEMO RECORDER
----------------------------------------
'Spacebar'  : start/stop demo recording
'I'         : get current demo index
'S'         : save recorded demo
'C'         : cancel recorded demo
'Ctrl-C'    : quit
----------------------------------------
"""

if __name__ == "__main__":
    rospy.init_node("demo_recorder")

    # Load params
    folder_path = os.environ["HOME"]
    cameras = []
    tags = []
    if rospy.has_param("~self.folder_path"):
        folder_path = rospy.get_param("~self.folder_path")
    if rospy.has_param("~cameras"):
        cameras = rospy.get_param("~cameras")
    if rospy.has_param("~tags"):
        tags = rospy.get_param("~tags")

    dr = DemoRecorder(folder_path, cameras, tags)

    menu_thread = threading.Thread(target=terminal_menu, args=(dr, menu_str, 0.5))
    menu_thread.start()

    rospy.spin()

    menu_thread.join()
