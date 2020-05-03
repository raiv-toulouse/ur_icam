#!/usr/bin/env python
# coding: utf-8

import rospy
from ur_icam_description.vacuum_gripper_set import VacuumGripperSet

#
# Un ensemble de 3x3 Grippers
#
rospy.init_node("gripper", anonymous=False)
gripperSet = VacuumGripperSet(9)  # Une matrice 3x3 grippers
rospy.spin()