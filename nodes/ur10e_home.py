#!/usr/bin/env python3
import math

import numpy as np
import rospy
from std_msgs.msg import Bool
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


HOME = [83.57, -59.07, 111.36, -145.45, 270.09, -143.31]
HOME = [math.radians(j) for j in HOME]

# Init the UR10e position
ur10e_actual_pos = []

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


# Define a callback function for obtaining the actual position of UR10e
def ur10e_state_callback(ur10e_state):
    global ur10e_actual_pos

    ur10e_actual_pos = list(ur10e_state.actual.positions)


if __name__ == "__main__":
    rospy.init_node("ur10e_control_node", anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # Obtain the actual position of UR10e
        ur10e_state_sub = rospy.Subscriber(
            "/scaled_pos_joint_traj_controller/state",
            JointTrajectoryControllerState,
            ur10e_state_callback,
        )

        # Check if the UR10e pose is obtained
        if not len(ur10e_actual_pos):
            continue

        # Check if the UR10e arrives the desired position
        ur10e_pose_diff = np.array(ur10e_actual_pos) - np.array(HOME)
        if np.sum(np.absolute(ur10e_pose_diff)) < 0.01:
            rospy.loginfo("UR10e arrives at HOME!")
            rate.sleep()
            continue
        
        # Move UR10e to the given HOME
        if abs(ur10e_actual_pos[1] - HOME[1]) >= 0.3:
            if ur10e_actual_pos[3] < -4.4 and abs(ur10e_actual_pos[3] + 5) >= 0.01:
                # Move wrist 1 to ensure the safe gap between the camera and door
                joint_point.positions = ur10e_actual_pos[:3] + [-5] + ur10e_actual_pos[4:]
            else:
                # Move the shoulder
                joint_point.positions = [ur10e_actual_pos[0], HOME[1]] + ur10e_actual_pos[2:]
        else:
            # Move all joints
            joint_point.positions = HOME

        joint_point.time_from_start.secs = 2
        joint_msg.points = [joint_point]
        joint_msg.header.stamp = rospy.Time.now()
        joint_pub.publish(joint_msg)
        rate.sleep()
