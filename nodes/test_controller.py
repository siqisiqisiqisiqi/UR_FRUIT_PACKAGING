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
from std_msgs.msg import String, Int16
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

from zed_3D_detection.msg import Box3d


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


class TrajectoryClient(unittest.TestCase):
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("test_move")

        # timeout = rospy.Duration(5)
        # self.switch_srv = rospy.ServiceProxy(
        #     "controller_manager/switch_controller", SwitchController
        # )
        # self.load_srv = rospy.ServiceProxy(
        #     "controller_manager/load_controller", LoadController)
        # self.list_srv = rospy.ServiceProxy(
        #     "controller_manager/list_controllers", ListControllers)

        # try:
        #     self.switch_srv.wait_for_service(timeout.to_sec())
        # except rospy.exceptions.ROSException as err:
        #     rospy.logerr(
        #         "Could not reach controller switch service. Msg: {}".format(err))
        #     sys.exit(-1)

        self.cartesian_trajectory_controller = ALL_CONTROLLERS[6]

        # Init corners subscribers
        rospy.Subscriber("/corners_test", Box3d, self.get_corners_data)

        # Init robot IO set
        rospy.wait_for_service('/ur_hardware_interface/set_io')
        rospy.Subscriber(
            "/ur_hardware_interface/io_states",
            IOStates,
            self.ur10e_DI_callback,
        )

        # receive the containers type
        rospy.Subscriber("chatter", String, self.box_type_call_back)

        # publish the robot status
        self.pub = rospy.Publisher('robot_status', Int16, queue_size=10)

        # soft gripper service initilization
        self.set_io = rospy.ServiceProxy(
            '/ur_hardware_interface/set_io', SetIO)
        # change the configuration of the gripper to open
        self.set_io(fun=1, pin=0, state=0)

        # Init the configuration of the robot
        self.home = geometry_msgs.Vector3(0.15, -0.75, 0.4)
        self.orientation = geometry_msgs.Quaternion(1.0, 0, 0, 0)

        # Get the box position
        # path_file = rospy.get_param("~scanning_path")
        # with open(f"{parent}/config/{path_file}", "r") as f:
        #     box_info = yaml.safe_load(f)
        # self.position = box_info["Position"]
        # self.capacity = box_info["Capacity"][0]

        self.M = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.T = np.array([[0.205], [-0.936], [-0.275]])

        # distance between the peach upper face center and tools
        self.offset = 0.15
        # gripper force sensor status
        self.DI_state = False
        # 3d object model result
        self.corner_data = None
        # fruit container's type
        self.box_type = None
        self.rate = rospy.Rate(10)

    # gripper force sensor reply
    def ur10e_DI_callback(self, ur10e_DI_state):
        self.DI_state = ur10e_DI_state.digital_in_states[4].state

    def box_type_call_back(self, data):
        self.box_type = data.data
        path_file = f"{self.box_type}.yaml"
        with open(f"{parent}/config/{path_file}", "r") as f:
            box_info = yaml.safe_load(f)
        self.position = box_info["Position"]
        self.capacity = box_info["Capacity"][0]

    def get_corners_data(self, data):
        corner_data = []
        try:
            for b in data.corners_data:
                self.corners = np.array(b.data).reshape((8, 3))
                corner_data.append(self.corners)
            # CHANGE THE UNIT FROM CENTIMETER TO METER
            self.corner_data = np.array(corner_data) / 100
            self.num = self.corner_data.shape[0]
        except:
            rospy.loginfo(f"Not detect the peach!")

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

    def calculate_target(self):
        # get the first peach location
        for i in range(self.num):
            corner = self.corner_data[i]
            corner_up = corner[:4, :]
            center_up = np.mean(corner_up, axis=0,
                                keepdims=True).T  # shape (3,1)
            target = self.M @ center_up + self.T
            target[2, 0] = target[2, 0] + self.offset
            if target[0, 0] > -0.3:
                return target
        return None

    def run(self):

        idx = 1
        status = 0

        self.initiate_cartesian_trajectory()
        rospy.sleep(2)
        self.go_home()
        while not rospy.is_shutdown():
            self.pub.publish(status)

            if self.corner_data is not None and idx <= self.capacity \
                and self.box_type is not None:

                status = 1
                target = self.calculate_target()

                if target is not None:
                    self.send_cartesian_trajectory(target.squeeze())
                    rospy.sleep(0.2)
                    self.set_io(fun=1, pin=0, state=1)  # close the gripper
                    while self.DI_state is not True:  # wait until the gripper is closed
                        rospy.loginfo("wait for the gripper to close!")
                        continue
                    rospy.loginfo("gripper closed")
                    self.go_home()
                    self.send_cartesian_trajectory(self.position[idx])
                    self.set_io(fun=1, pin=0, state=0)  # open the gripper
                    self.DI_state = False
                    rospy.sleep(0.2)
                    self.go_home()
                    rospy.loginfo("Finish the movement.")
                    idx = idx + 1
                    
                if idx == self.capacity:
                    status = 2
                    rospy.loginfo("Finish the task!")
                    self.box_type = None
                    idx = 1

            self.rate.sleep()

    # def switch_on_controller(self, target_controller):
    #     """Activates the desired controller and stops all others from the predefined list above"""
    #     other_controllers = (
    #         ALL_CONTROLLERS
    #     )

    #     other_controllers.remove(target_controller)

    #     srv = ListControllersRequest()
    #     response = self.list_srv(srv)
    #     for controller in response.controller:
    #         if controller.name == target_controller and controller.state == "running":
    #             return

    #     srv = LoadControllerRequest()
    #     srv.name = target_controller
    #     self.load_srv(srv)

    #     srv = SwitchControllerRequest()
    #     srv.stop_controllers = other_controllers
    #     srv.start_controllers = [target_controller]
    #     srv.strictness = SwitchControllerRequest.BEST_EFFORT
    #     self.switch_srv(srv)


if __name__ == "__main__":
    client = TrajectoryClient()
    client.run()
