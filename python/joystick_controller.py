#!/usr/bin/env python3

from cartesian_interface.pyci_all import *
import rospy
from sensor_msgs.msg import Joy
import sys
import numpy as np
from std_msgs.msg import Float32

class JoystickController:
    def __init__(self):
        rospy.init_node("joystick_controller")

        self.joy_topic = rospy.get_param("~joy_topic", "/tiago_dual_cartesio/joy")
        self.tf_prefix = rospy.get_param("~tf_prefix", "ci")
        self.lin_scale = rospy.get_param("~lin_scale", 0.2)
        self.ang_scale = rospy.get_param("~ang_scale", 1.0)

        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)
        self.ros_control_publisher = rospy.Publisher("/ros_control_bridge/time_from_start", Float32, queue_size=1)

        self.ros_control_publisher.publish(0.6)

        rospy.loginfo("Joystick controller initialized, Settings:")
        rospy.loginfo("Joy topic: %s", self.joy_topic)
        rospy.loginfo("TF prefix: %s", self.tf_prefix)
        rospy.loginfo("Linear scale: %s", self.lin_scale)
        rospy.loginfo("Angular scale: %s", self.ang_scale)

        try:
            self.cli = pyci.CartesianInterfaceRos()
        except Exception as e:
            rospy.logerr("No Cartesio server found. Shutting down.")
            sys.exit()

        self.task = self.cli.getTask("base_link")

        self.task.setControlMode(pyci.ControlType.Velocity)

    def joy_callback(self, msg):
        vel_x = msg.axes[1] * self.lin_scale
        vel_y = msg.axes[0] * self.lin_scale
        vel_ang = msg.axes[3] * self.ang_scale

        deadman_button = msg.buttons[7]
        
        if deadman_button == 1:
            ref = np.array([vel_x, vel_y, 0.0, 0.0, 0.0, vel_ang])
            
            H = self.cli.getPoseFromTf(
                self.tf_prefix + "/" + self.task.getName(),
                self.tf_prefix + "/" + self.task.getBaseLink(),
            )

            adj = np.block(
                [
                    [H.linear, np.zeros((3, 3))],
                    [np.zeros((3, 3)), H.linear],
                ]
            )

            ref = adj @ ref

            self.task.setVelocityReference(ref)




if __name__ == "__main__":
    jc = JoystickController()
    rospy.spin()


        