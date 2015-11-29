#!/usr/bin/env python
import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo():

 def __init__(self):
     self.node_name = "cv_bridge_demo"
     #Initialize the ros node
     rospy.init_node(self.node_name)
     # What we do during shutdown
     # Create the OpenCV display window for the RGB image
     self.cv_window_name = self.node_name
     cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
     cv.MoveWindow(self.cv_window_name, 25, 75)
     # Create the cv_bridge object
     self.bridge = CvBridge()
     # Subscribe to the camera image and depth topics and set
     # the appropriate callbacks
     self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback)
     self.image_pub = rospy.Publisher("image_mask", Image)
     rospy.loginfo("Waiting for image topics...")


 def image_callback(self, ros_image):

     try:
      frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
     except CvBridgeError, e:
      print e
     # require Numpy arrays.
     frame = np.array(frame, dtype=np.uint8)
     display_image = self.process_image(frame)
     # Display the image.
     cv2.imshow(self.node_name, display_image)
     # Process any keyboard commands
     self.keystroke = cv.WaitKey(5)
     if 32 <= self.keystroke and self.keystroke < 128:
          cc = chr(self.keystroke).lower()
     if cc == 'q':
       # The user has press the q key, so exit
          rospy.signal_shutdown("User hit q key to quit.")

 def process_image(self, frame):
     
     # hsv color segmentation
     imghsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     lower_blue=np.array([112,122,89])
     higher_blue=np.array([132,255,255])
     mask=cv2.inRange(imghsv,lower_blue,higher_blue)           
     
     # morphological filtering noise removal
     kernel = np.ones((5,5),np.uint8)
     opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) 
     try:
      opening_1 = self.bridge.cv2_to_imgmsg(opening, "8UC1")
     except CvBridgeError, e:
      print e
     self.image_pub.publish(opening_1) 
     return opening
     

def main(args):
     try:
      cvBridgeDemo()
      rospy.spin()
     except KeyboardInterrupt:
      print "Shutting down vision node."
      cv.DestroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)