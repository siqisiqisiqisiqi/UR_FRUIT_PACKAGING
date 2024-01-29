#!/usr/bin/env python3
import sys
import os
import math

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent + "/src")

import numpy as np
import yaml
import rospy
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint



# Init the UR10e position
HOME = [83.57, -59.07, 111.36, -145.45, 270.09, -143.31]
HOME = [math.radians(j) for j in HOME]
ur10e_actual_pos = np.array(HOME)


# Define a callback function for obtaining the actual position of UR10e
def ur10e_state_callback(ur10e_state):
    global ur10e_actual_pos

    ur10e_actual_pos = np.array(ur10e_state.actual.positions)

def ur10e_DI_callback(ur10e_DI_state):
    global DI_state
    DI_state = ur10e_DI_state.digital_in_states[4].state
    # rospy.loginfo(f"ur10e DI state is {DI_state}")

if __name__ == "__main__":
    DI_state = False
    rospy.init_node("inspection_node", anonymous=True)
    rate = rospy.Rate(int(rospy.get_param("~publish_rate")))
    # Service subscribe
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    ur10e_state_sub = rospy.Subscriber(
        "/ur_hardware_interface/io_states",
        IOStates,
        ur10e_DI_callback,
    )
    set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

    # change the configuration of the gripper
    set_io(fun = 1,pin = 0,state = 0)
    # Read the designed UR10e path
    path_file = rospy.get_param("~scanning_path")
    with open(f"{parent}/config/{path_file}", "r") as f:
        scan_path = yaml.safe_load(f)

    position = scan_path["trajectory"]
    trajectory = []
    index = [1,2,1,3,4,3,1]
    for i in index:
        trajectory.append(position[i])
    traj_np = np.array(trajectory)

    rospy.sleep(1)

    # Init instances for publishing Trajectory Msg
    joint_msg = JointTrajectory()
    joint_msg.header.frame_id = ""
    joint_msg.joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    joint_point = JointTrajectoryPoint()
    joint_pub = rospy.Publisher(
        "/scaled_pos_joint_traj_controller/command",
        JointTrajectory,
        queue_size=1,
    )

    # Start path test
    idx = 0
    while not rospy.is_shutdown():
        # Terminate the program
        if idx >= len(index):
            rospy.loginfo("Finish the pick and place task!")
            rate.sleep()
            continue

        # Move the UR10e to the next position
        waypoint = [math.radians(j) for j in traj_np[idx]]

        if idx == 0:
            joint_point.time_from_start.secs = 2
            joint_point.time_from_start.nsecs = 0
        else:
            joint_point.time_from_start.secs = 0
            joint_point.time_from_start.nsecs = int(2e8)
        joint_msg.header.stamp = rospy.Time.now()
        joint_point.positions = waypoint
        joint_msg.points = [joint_point]
        joint_pub.publish(joint_msg)

        # Obtain the actual position of UR10e
        ur10e_state_sub = rospy.Subscriber(
            "/scaled_pos_joint_traj_controller/state",
            JointTrajectoryControllerState,
            ur10e_state_callback,
        )

        # Check if the UR10e arrives the desired position
        ur10e_pos_diff = ur10e_actual_pos - np.array(waypoint)

        # Start the defect detection process
        if np.sum(np.absolute(ur10e_pos_diff)) < 0.001:
            # Load the next UR10e position
            # input("Press ENTER to test the next scan point...")
            idx += 1
            if idx == 2:
                rospy.sleep(2)
                set_io(fun = 1,pin = 0,state = 1) # close the gripper
                while DI_state is not True: # wait until the gripper is closed
                    rospy.loginfo("wait for the gripper to close!")
                    continue
                rospy.sleep(1)
            if idx == 5:
                set_io(fun = 1,pin = 0,state = 0)
                rospy.sleep(2)
                DI_state = False
        rate.sleep()
