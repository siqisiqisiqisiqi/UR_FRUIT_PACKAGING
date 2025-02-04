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
from ur_msgs.srv import SetIO
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float32MultiArray

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
        rospy.init_node("green_onion_picking")

        self.cartesian_trajectory_controller = ALL_CONTROLLERS[6]

        # Init robot IO set
        rospy.wait_for_service('/ur_hardware_interface/set_io')

        # soft gripper service initilization
        self.set_io = rospy.ServiceProxy(
            '/ur_hardware_interface/set_io', SetIO)
        # change the configuration of the gripper to open
        self.set_io(fun=1, pin=0, state=0)

        # publish the subscribe
        self.pub = rospy.Publisher('robot_status', Int16, queue_size=10)
        rospy.Subscriber("/grasp_pose", Float32MultiArray, self.get_grasp_pose)

        # Init the configuration of the robot
        self.home = geometry_msgs.Vector3(0.15, -0.81, 0.23)  # (150, 810, 231)
        # RX, RY, RZ in degrees rz range(-45, 135)
        euler_angles = (180, 0, -45)
        q = euler_to_quaternion(*euler_angles)  # X, Y, Z, W
        self.home_orient = geometry_msgs.Quaternion(
            q[0], q[1], q[2], q[3])  # ()radian

        # translation between robot and world frame
        self.M = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        self.T = np.array([[0.465], [-0.890], [-0.180]])

        # gripper force sensor status
        self.grasp_position = None
        self.grasp_yaw = None
        self.pick_offset = np.array([0, 0, 0.2])
        self.tool_offset = np.array([0, 0, 0.085])
        self.place_position = np.array([0.63, 0.12, 0.2])

        self.rate = rospy.Rate(10)

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

    def send_cartesian_trajectory(self, p, yaw):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        # self.switch_on_controller(self.cartesian_trajectory_controller)
        yaw = yaw * 180 / np.pi
        euler_angles = (180, 0, -45 + yaw)
        q = euler_to_quaternion(*euler_angles)  # X, Y, Z, W
        orientation = geometry_msgs.Quaternion(q[0], q[1], q[2], q[3])

        goal = FollowCartesianTrajectoryGoal()
        rospy.loginfo(f"p value is {p}")

        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(p[0], p[1], p[2]), orientation
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
            geometry_msgs.Pose(self.home, self.home_orient),
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

    def get_grasp_pose(self, data):
        """_summary_

        Parameters
        ----------
        data : Float32MultiArray
            [x(mm),y(mm),z(mm),yaw(radian)]
        """
        rospy.loginfo(f"data value is {data.data}")
        try:
            grasp_position = np.array(data.data[:3]) * 1e-3  # convert from mm to m
            self.grasp_yaw = data.data[-1]
            # The length of the gripper is 0.18m
            grasp_position[0] += 0.09 * np.cos(self.grasp_yaw)
            grasp_position[1] += 0.09 * np.sin(self.grasp_yaw)
            grasp_position = grasp_position.reshape((3, 1))
            grasp_position = self.M @ grasp_position + self.T
            rospy.loginfo(f"grasp position is {grasp_position}")
            self.grasp_position = grasp_position.squeeze() + self.tool_offset
        except:
            self.grasp_yaw = None
            self.grasp_position = None
            rospy.loginfo(f"No suitable green onion to pick!")

    def pick_and_place(self):
        rospy.loginfo(f"self.grasp_position is {self.grasp_position}")
        self.grasp_position_offset = self.grasp_position + self.pick_offset
        self.send_cartesian_trajectory(
            self.grasp_position_offset, self.grasp_yaw)
        rospy.sleep(0.5)
        self.send_cartesian_trajectory(self.grasp_position, self.grasp_yaw)
        rospy.sleep(0.5)
        self.set_io(fun=1, pin=0, state=1)  # close the gripper
        rospy.sleep(0.5)
        rospy.loginfo("gripper closed")
        self.go_home()
        self.send_cartesian_trajectory(self.place_position, self.grasp_yaw)
        self.set_io(fun=1, pin=0, state=0)  # open the gripper
        rospy.sleep(0.5)
        self.go_home()
        rospy.loginfo("Finish the movement.")

    def run(self):
        status = 0
        self.initiate_cartesian_trajectory()
        rospy.sleep(2)
        self.go_home()
        while not rospy.is_shutdown():
            if self.grasp_yaw is not None and self.grasp_position is not None:
                self.pick_and_place()
                break
            self.pub.publish(status)
            self.rate.sleep()


if __name__ == "__main__":
    client = TrajectoryClient()
    client.run()
