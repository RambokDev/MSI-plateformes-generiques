#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import logging


def gripper_state(state: bool):
    gripper = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    if state:
        gripper_str = "open"
    elif not state:
        gripper_str = "close"
    logging.info(gripper_str)
    try:
        rospy.loginfo(gripper_str)
        gripper.publish(gripper_str)
        rate.sleep()
    except rospy.ServiceException as exc:
        logging.error("Error {}".format(gripper_str) + str(exc))


if __name__ == '__main__':
    try:
        gripper_state(state=True)
    except rospy.ROSInterruptException:
        pass
