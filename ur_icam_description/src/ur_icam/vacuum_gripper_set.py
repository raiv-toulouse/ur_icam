#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import Bool
from vacuum_gripper import VacuumGripper

class VacuumGripperSet:
    def __init__(self,nbGrippers):
        self.lesGrippers = []
        for i in range(nbGrippers):
            self.lesGrippers.append(VacuumGripper(i))
        gripper_sub = rospy.Subscriber('/ur5/vacuum_gripper/grasp', Bool, self.grasp, queue_size=1)

    def grasp(self,msg):
        if msg.data:
            for g in self.lesGrippers:
                g.gripper_on()
        else:
            for g in self.lesGrippers:
                g.gripper_off()
       
#
# Permet de positionner le robot à un endroit précis à l'aide de coord cartésiennes
#
if __name__ == '__main__':
    rospy.init_node("gripper", anonymous=False)
    gripperSet = VacuumGripperSet(9)  # Une matrice 3x3 grippers
    rospy.spin()
