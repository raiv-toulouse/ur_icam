#!/usr/bin/env python
# coding: utf-8

import copy
import sys
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_commander
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_commander.conversions import pose_to_list


class RobotUR(object):
    def __init__(self):
        super(RobotUR, self).__init__()
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_UR', anonymous=True)
        # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        # kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()
        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to a planning group (group of joints).  The group is the primary
        # arm joints in the UR robot, so we set the group's name to "manipulator".
        # This interface can be used to plan and execute motions:
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def get_current_pose(self):
        return self.move_group.get_current_pose()

    def get_current_joint(self):
        return self.move_group.get_current_joint_values()

    def deconnecter(self):
        moveit_commander.roscpp_shutdown()

    def setPose(self, x, y, z, phi, theta, psi):
        orient = Quaternion(quaternion_from_euler(phi, theta, psi))
        pose = Pose(Point(x, y, z), orient)
        self.move_group.set_pose_target(pose)
        self.move_group.go(True)
        self.move_group.stop()

    # Planning to a Joint Goal
    def go_to_joint_state(self, joints_goal):
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joints_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        # On teste si l'on a atteind l'objectif
        current_joints = self.move_group.get_current_joint_values()
        return self.all_close(joints_goal, current_joints, 0.01)

        # Planning to a cartesian goal


    def go_to_pose_goal(self, pose_goal):
        # We can plan a motion for this group to a desired pose for the end-effector:
        self.move_group.set_pose_target(pose_goal)
        # Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return self.all_close(pose_goal, current_pose, 0.01)

        # Execute a cartesian path throw a list of waypoints


    def exec_cartesian_path(self, waypoints):
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()


    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)
        elif type(goal) is Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
        return True


#
#  Démo des différentes fonctions du robot
#
if __name__ == '__main__':
    myRobot = RobotUR()
    # Getting Basic Information
    # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = myRobot.move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame
    # We can also print the name of the end-effector link for this group:
    eef_link = myRobot.move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link
    # We can get a list of all the groups in the robot:
    group_names = myRobot.robot.get_group_names()
    print "============ Available Planning Groups:", myRobot.robot.get_group_names()
    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot current pose"
    print myRobot.get_current_pose()
    print "============ Printing robot state"
    print myRobot.robot.get_current_state()
    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    # On teste le positionnement par rapprot à des coordonnées angulaires
    objectifAtteint = myRobot.go_to_joint_state(
        [0, -pi / 4, 0, -pi / 2, 0, pi / 3])
    if objectifAtteint:
        print("L'objectif est atteint")
    else:
        print("On n'est pas sur l'objectif")
    # On teste le positionnement par rapport à des coordonnées cartésiennes
    print("Test de go_to_pose_goal")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    myRobot.go_to_pose_goal(pose_goal)
    # On teste le déplacement en coord cartésiennes entre différents waypoints
    print("Test de exec_cartesian_path")
    waypoints = []
    wpose = myRobot.get_current_pose().pose
    wpose.position.z += 0.1  # First move up (z)
    wpose.position.y += 0.1  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y -= 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    myRobot.exec_cartesian_path(waypoints)
    print("Fin")
