#!/usr/bin/env python
# coding: utf-8
import rospy
from std_srvs.srv import Empty

#
#  Prend une photo à l'aide du service 'record_image' fourni par camera.py
#
if __name__ == '__main__':
    rospy.loginfo("On prend une photo")
    rospy.wait_for_service('record_image')
    try:
        srv_record = rospy.ServiceProxy('record_image', Empty)
        srv_record()  # Appel au service
    except rospy.ServiceException, e:
        rospy.logerr("Service record_image failed")

