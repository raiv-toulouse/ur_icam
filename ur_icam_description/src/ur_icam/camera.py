# coding: utf-8

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
from std_srvs.srv import Empty,EmptyResponse

class Camera:
  def __init__(self,repImage):
    self.indImage = 0
    self.repImage = repImage
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw',
                                      Image, self.display_image)
    s = rospy.Service('record_image', Empty, self.record_image)
    rospy.loginfo("Servide record_image *******************************")


  def record_image(self,msg):
      cv2.imwrite(self.repImage+'/img{:04d}.png'.format(self.indImage),self.image)
      self.indImage+=1
      return EmptyResponse()


  def display_image(self, msg):
    self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    cv2.namedWindow("window", 1)
    cv2.imshow("window", self.image)
    cv2.waitKey(5)
