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
            self.on()
        else:
            self.off()

    def on(self):
        for g in self.lesGrippers:
            g.gripper_on()

    def off(self):
        for g in self.lesGrippers:
            g.gripper_off()