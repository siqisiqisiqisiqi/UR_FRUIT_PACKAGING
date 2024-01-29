#!/usr/bin/env python3
import math

import numpy as np
import rospy
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO


ur10e_DO_state = IOStates()

def ur10e_DI_callback(ur10e_DI_state):
    DI_state = ur10e_DI_state.digital_in_states[4].state
    rospy.loginfo(f"ur10e DI state is {DI_state}")


if __name__ == "__main__":
    rospy.init_node("ur10e_gripper_control_node", anonymous=True)
    rate = rospy.Rate(20)
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    ur10e_state_sub = rospy.Subscriber(
            "/ur_hardware_interface/io_states",
            IOStates,
            ur10e_DI_callback,
        )
    set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    set_io(fun = 1,pin = 1,state = 1)
    while not rospy.is_shutdown():
        
        rate.sleep()
