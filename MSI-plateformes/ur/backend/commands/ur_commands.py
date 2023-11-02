#!/usr/bin/python3
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


def convert_deg_to_rad(tab):
    """
    this function allow you to convert deg to rad.
    @param: tab
    @returns: list     A list named res
    """
    res = []
    for i in tab:
        res.append(i * pi / 180)
    return res


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)
        ## Instantiate a `RobotCommander` object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).
        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        ## Create a `DisplayTrajectory` ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        ## Getting Basic Information
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)
        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        # Misc variables
        self.robot = robot
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        # init variable préhension
        self.stop_movement = False

    def event_callback(self, data):
        if data.digital_in_states[0].state == True:
            print(data.digital_in_states[0].state)
            self.stop_movement = True

    def prehension(self, pose):
        """
        Partie en développement au moment de l'arrêt du MSI S1 2022/2023
        """

        # Initialize node
        # rospy.init_node('moveit_example')

        # Initialize moveit_commander
        # moveit_commander.roscpp_initialize(sys.argv)

        # Create group commander
        # group = moveit_commander.MoveGroupCommander("arm")

        # Create subscriber
        rospy.Subscriber("ur_hardware_interface/io_states", IOStates, self.event_callback)
        # print(msg.digital_in_states[0].state)

        # Create publisher
        # move_pub = rospy.Publisher("move_topic", String, queue_size=10)

        # Set target pose or joint values
        self.move_group.set_pose_target(pose)
        # or
        # self.move_group.set_joint_value_target(...)

        # Plan and execute movement
        success = self.move_group.go(wait=True)
        print('premier success', success)

        # Wait for movement to finish or for stop signal
        # while not self.stop_movement and not self.move_group.get_current_state() == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
        #     rospy.sleep(0.1)
        #     print(self.move_group.get_current_state())

        while self.stop_movement == False:  # and success == False:
            # print('in')
            rospy.sleep(0.1)

        print('out')
        self.stop_movement = False

        # Stop movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Send stop signal via publisher
        # self.move_pub.publish("stopped")

    def go_to_joint_state(self, joint_goal):
        """
        Go to a joint state
        Example : [0, -2*pi/8, 0, -2*pi/4, 0, 2*pi/6]
        @param: joint_goal       A list of floats in radian, a value per joint
        """

        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        print(joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)
        print('1')
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal):
        """
        Go to a pose goal in the end-effector frame
        (To change the method set_pose_reference_frame(), to go to a position with any orientation see set_position_target())
        Example : list of 6 floats [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats (see quaternions) [x, y, z, qx, qy, qz, qw]
        Example : [0.4, 0.1, 0.4, 0, 0, 0, 1]
        @param: pose       A list of floats
        """

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        liste = pose_goal.copy()
        pose_goal = geometry_msgs.msg.Pose()
        if len(liste) == 7:
            pose_goal.position.x = liste[0]
            pose_goal.position.y = liste[1]
            pose_goal.position.z = liste[2]
            pose_goal.orientation.x = liste[3]
            pose_goal.orientation.y = liste[4]
            pose_goal.orientation.z = liste[5]
            pose_goal.orientation.w = liste[6]
        elif len(liste) == 6:
            pose_goal.position.x = liste[0]
            pose_goal.position.y = liste[1]
            pose_goal.position.z = liste[2]
            q = tf.transformations.quaternion_from_euler(liste[3], liste[4], liste[5])
            pose_goal.orientation.x = q[0]
            pose_goal.orientation.y = q[1]
            pose_goal.orientation.z = q[2]
            pose_goal.orientation.w = q[3]

        self.move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        print(current_pose)
        print(f"Result for all_close function : {all_close(pose_goal, current_pose, 0.01)}")
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, waypoints):
        """
        Compute a path from a list of pose
        Example of pose : list of 6 floats [x, y, z, rot_x, rot_y, rot_z] or a list of 7 floats (see quaternions) [x, y, z, qx, qy, qz, qw]
        Example of path :   [[0.4, 0.1, 0.4, 0, 0, 0, 1],
                            [0.4, 0.3, 0.3, 0, 0, 0, 1],
                            [0.1, 0.1, 0.4, 0, 0, 0, 1],
                            [0, 0.1, 0.4, 0, 0, 0, 1]]
        @param: waypoints       A two dim array of float
        """
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        for i in range(len(waypoints)):
            pose_goal = geometry_msgs.msg.Pose()
            if len(waypoints[i]) == 7:
                pose_goal.position.x = waypoints[i][0]
                pose_goal.position.y = waypoints[i][1]
                pose_goal.position.z = waypoints[i][2]
                pose_goal.orientation.x = waypoints[i][3]
                pose_goal.orientation.y = waypoints[i][4]
                pose_goal.orientation.z = waypoints[i][5]
                pose_goal.orientation.w = waypoints[i][6]
            elif len(waypoints[i]) == 6:
                pose_goal.position.x = waypoints[i][0]
                pose_goal.position.y = waypoints[i][1]
                pose_goal.position.z = waypoints[i][2]
                q = tf.transformations.quaternion_from_euler(waypoints[i][3], waypoints[i][4], waypoints[i][5])
                pose_goal.orientation.x = q[0]
                pose_goal.orientation.y = q[1]
                pose_goal.orientation.z = q[2]
                pose_goal.orientation.w = q[3]
            waypoints[i] = pose_goal

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
        """
        Execute a planned path
        @param: plan       A plan path from the method compute_cartesian_path()
        """
        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        print("Setting up the moveit_commander ...")
        tutorial = MoveGroupPythonInterface()

        input("============ Press `Enter` to execute a movement using a joint state goal ...")
        print("go_to_joint_state([0, -2*pi/8, 0, -2*pi/4, 0, 2*pi/6])")
        tutorial.go_to_joint_state([0, -2 * pi / 8, 0, -2 * pi / 4, 0, 2 * pi / 6])

        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # print("go_to_pose_goal([0, -0.8, 0, -1.6, 0, 1]))")
        # tutorial.go_to_pose_goal([1, 1, 1, 0, 0, 0, 1])

        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # print("plan_cartesian_path([[0.4, 0.1, 0.4, 0, 0, 0, 1], [0.4, 0.3, 0.3, 0, 0, 0, 1], [0.1, 0.1, 0.4, 0, 0, 0, 1], [0, 0.1, 0.4, 0, 0, 0, 1]])")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(    [[0.4, 0.1, 0.4, 0, 0, 0, 1],
        #                                                             [0.4, 0.3, 0.3, 0, 0, 0, 1],
        #                                                             [0.1, 0.1, 0.4, 0, 0, 0, 1],
        #                                                             [0, 0.1, 0.4, 0, 0, 0, 1]])

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
