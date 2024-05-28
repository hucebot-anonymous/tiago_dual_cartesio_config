#!/usr/bin/env python

from cartesian_interface.pyci_all import *
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

ci = pyci.CartesianInterfaceRos()
ee_left_pose_initial, _, _ = ci.getPoseReference('gripper_left_grasping_frame')
reference_publisher = rospy.Publisher('/cartesian/gripper_left_grasping_frame/reference', PoseStamped, queue_size=10)
initial_ref = None
def callback(data: PoseStamped):
    r_init = R.from_quat([initial_ref.pose.orientation.x, initial_ref.pose.orientation.y, initial_ref.pose.orientation.z, initial_ref.pose.orientation.w])
    data_m_init = Affine3()
    data_m_init.translation = np.array([initial_ref.pose.position.x, initial_ref.pose.position.y, initial_ref.pose.position.z])
    data_m_init.linear = r_init.as_matrix()

    r = R.from_quat([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    data_m = Affine3()
    data_m.translation = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
    data_m.linear = r.as_matrix()

    ref_m =  ee_left_pose_initial * data_m_init.inverse() * data_m

    reference = PoseStamped()
    reference.pose.position.x = ref_m.translation[0]
    reference.pose.position.y = ref_m.translation[1]
    reference.pose.position.z = ref_m.translation[2]
    reference.pose.orientation.x = data.pose.orientation.x
    reference.pose.orientation.y = data.pose.orientation.y
    reference.pose.orientation.z = data.pose.orientation.z
    reference.pose.orientation.w = data.pose.orientation.w

    reference.header.frame_id = "ci/world"
    reference.header.stamp = rospy.get_rostime()
    reference_publisher.publish(reference)

if __name__ == '__main__':
    rospy.init_node('teleop_bridge', anonymous=True)
    initial_ref = rospy.wait_for_message("pos", PoseStamped, timeout=5)
    rospy.Subscriber("pos", PoseStamped, callback)

    rospy.spin()