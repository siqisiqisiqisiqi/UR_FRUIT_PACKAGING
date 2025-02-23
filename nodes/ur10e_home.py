#!/usr/bin/env python3

import sys
import unittest
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)

import rospy
import yaml
import actionlib
import numpy as np
from std_msgs.msg import Int16
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from scipy.spatial.transform import Rotation as R


ALL_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "vel_joint_traj_controller",
    "joint_group_vel_controller",
    "forward_joint_traj_controller",
    "forward_cartesian_traj_controller",
    "twist_controller",
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
]


def euler_to_quaternion(rx, ry, rz, degrees=True):
    """
    Convert Euler angles to a quaternion using scipy.

    Args:
    rx, ry, rz (float): Euler angles for roll (X-axis), pitch (Y-axis), and yaw (Z-axis).
    degrees (bool): Whether the input angles are in degrees. Default is True.

    Returns:
    tuple: Quaternion (qx, qy, qz, qw).
    """
    # Create rotation object
    rotation = R.from_euler('xyz', [rx, ry, rz], degrees=degrees)

    # Convert to quaternion
    quaternion = rotation.as_quat()  # Format: [qx, qy, qz, qw]

    return tuple(quaternion)


class TrajectoryClient(unittest.TestCase):
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("test_move")

        self.cartesian_trajectory_controller = ALL_CONTROLLERS[6]

        # Init robot IO set
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        rospy.Subscriber(
            "/ur_hardware_interface/io_states",
            IOStates,
            self.ur10e_DI_callback,
        )

        # publish the robot status
        self.pub = rospy.Publisher('robot_status', Int16, queue_size=10)

        # soft gripper service initilization
        self.set_io = rospy.ServiceProxy(
            '/ur_hardware_interface/set_io', SetIO)
        # change the configuration of the gripper to open
        self.set_io(fun=1, pin=0, state=0)

        # Init the configuration of the robot
        self.home = geometry_msgs.Vector3(0.0, -0.80, 0.0)  # (150, 810, 231)

        # RX, RY, RZ in degrees rz range(-45, 135)
        euler_angles = (180, 0, -45)
        q = euler_to_quaternion(*euler_angles)  # X, Y, Z, W
        self.orientation = geometry_msgs.Quaternion(
            q[0], q[1], q[2], q[3])  # ()radian

        # gripper force sensor status
        self.DI_state = False
        self.rate = rospy.Rate(10)

    # gripper force sensor reply
    def ur10e_DI_callback(self, ur10e_DI_state):
        self.DI_state = ur10e_DI_state.digital_in_states[4].state

    def initiate_cartesian_trajectory(self):
        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(
                self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not self.trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

    def send_cartesian_trajectory(self, target):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        # self.switch_on_controller(self.cartesian_trajectory_controller)

        goal = FollowCartesianTrajectoryGoal()

        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(
                    target[0], target[1], target[2]), self.orientation
            )
        ]
        duration_list = [10.0] * len(pose_list)
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()

        rospy.loginfo(
            "Trajectory execution finished in state {}".format(result.error_code))

    def go_home(self):
        goal = FollowCartesianTrajectoryGoal()

        pose_list = [
            geometry_msgs.Pose(self.home, self.orientation),
        ]
        duration_list = [10.0] * len(pose_list)
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()

        rospy.loginfo(
            "Trajectory execution finished in state {}".format(result.error_code))

    def run(self):
        status = 0
        self.initiate_cartesian_trajectory()
        rospy.sleep(2)
        self.go_home()
        while not rospy.is_shutdown():
            self.pub.publish(status)
            self.rate.sleep()


if __name__ == "__main__":
    client = TrajectoryClient()
    client.run()
