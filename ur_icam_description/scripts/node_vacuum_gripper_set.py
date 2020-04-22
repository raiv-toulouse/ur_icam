#!/usr/bin/env python
# coding: utf-8

import rospy
from ur_icam.vacuum_gripper_set import VacuumGripperSet

#
# Un ensemble de 3x3 Grippers
#
if __name__ == '__main__':
    rospy.init_node("gripper", anonymous=False)
    gripperSet = VacuumGripperSet(9)  # Une matrice 3x3 grippers
    rospy.spin()
