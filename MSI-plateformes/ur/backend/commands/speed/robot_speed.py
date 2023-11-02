#!/usr/bin/env python3
import rospy
from ur_msgs.srv import SetSpeedSliderFraction, SetSpeedSliderFractionRequest, SetSpeedSliderFractionResponse


def speed_state(speed: int):
    """
    Change de speed slider of the ur Dashboard
    Example : 10
    @param: speed       An int that represent the speed in porcent
    """
    rospy.wait_for_service('/ur_hardware_interface/set_speed_slider')
    robot_speed_state = rospy.ServiceProxy('/ur_hardware_interface/set_speed_slider', SetSpeedSliderFraction)

    try:
        resp = robot_speed_state(speed)
        return resp.success, resp.message
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


if __name__ == '__main__':
    print("Set robot Speed")
    speed_state(speed=10)
