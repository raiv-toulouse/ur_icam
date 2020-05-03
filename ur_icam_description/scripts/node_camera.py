#!/usr/bin/env python
# coding: utf-8
from ur_icam_description.camera import Camera
import rospy

rospy.init_node('myCamera')
myCamera = Camera()
rospy.spin()
