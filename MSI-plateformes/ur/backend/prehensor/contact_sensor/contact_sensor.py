#!/usr/bin/env python3
import rospy
from ur_msgs.msg import IOStates


def contact_sensor():
    """
    This function is create in order to get the contact send state that we have implement next to the gripper
    """
    while True:
        rospy.init_node('listener_sensor')
        msg = rospy.wait_for_message('ur_hardware_interface/io_states', IOStates)
        print(msg.digital_in_states[7].state)


if __name__ == '__main__':
    print("contact Sensor gripper")
    contact_sensor()
