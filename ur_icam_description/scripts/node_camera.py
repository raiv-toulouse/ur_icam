#!/usr/bin/env python
# coding: utf-8
from ur_icam.camera import Camera
import rospy

rospy.init_node('myCamera')
myCamera = Camera('/home/philippe')
rospy.spin()
