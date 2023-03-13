#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import kinova_positional_control.srv as posctrl_srv

class imageConverter:

  # Class constructor
  def __init__(self):
    self.bridge = CvBridge()

    # Set flags
    self.isStarted = False
    self.recordResume = False

    # Subscribing
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)

    # Server
    rospy.Service('/video_recorder/resume_video', posctrl_srv.resume_record, self.resume_video_handler)
    rospy.Service('/video_recorder/init_video', posctrl_srv.init_record, self.init_video_handler)
    rospy.Service('/video_recorder/capture_image', posctrl_srv.capture_image, self.capture_image_handler)

    # Service
    self.resume_data_srv = rospy.ServiceProxy('/data_collector/resume_data', posctrl_srv.resume_record)


  # Camera topic callback
  def camera_callback(self, msg):

    if self.isStarted:
      try:
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
      except CvBridgeError as e:
        print(e)

        # Pause data collector
        self.resume_data_src(False)

      # Rotate the image
      self.cv_image_rot = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)

      if self.recordResume == True:
        # Optionally show the frame
        cv2.imshow("Image window", self.cv_image_rot)

        # Write the frame to the video file
        self.video_writer.write(self.cv_image_rot)
        cv2.waitKey(3)


  # Init recording service handler
  def init_video_handler(self, req):

    if req.request == True:
      self.isStarted = True
      self.videofile_name = req.filename

      # Check if the video already exists
      if (os.path.exists(self.videofile_name)):
        i = 1

        while (os.path.exists(self.videofile_name[:-4] + '(' + str(i) + ')' + self.videofile_name[-4:])):
            i = i + 1

        self.videofile_name = self.videofile_name[:-4] + '(' + str(i) + ')' + self.videofile_name[-4:]

      # Initialize the video recorder
      self.video_writer = cv2.VideoWriter(self.videofile_name, cv2.VideoWriter_fourcc(*'MJPG'), 25, (720, 1280))
      print("\nVideo recorder is initialized under " + self.videofile_name + "\n")

    else:
      self.isStarted = False

    return True


  # Resume recording service handler
  def resume_video_handler(self, req):

    if req.request == True:
      print("\nVideo recorder is resumed.\n")
      self.recordResume = True

    else:
      print("\nVideo recorder is paused.\n")
      self.recordResume = False

    return True
  

  # Capture an image handler
  def capture_image_handler(self, req):

    if self.isStarted == True:
      self.image_name = req.filename

      # Check if the image already exists
      if (os.path.exists(self.image_name)):
        i = 1

        while (os.path.exists(self.image_name[:-4] + '(' + str(i) + ')' + self.image_name[-4:])):
            i = i + 1

        self.image_name = self.image_name[:-4] + '(' + str(i) + ')' + self.image_name[-4:]

      # Save the image
      cv2.imwrite(self.image_name, self.cv_image_rot)

    return True
  

# This function is called when the node is shutting down
def node_shutdown():
    print("\nNode is shutting down...")

    # Pause data collector
    ic.resume_data_srv(False)

    print("\nNode has shut down.")


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('image_converter', anonymous=True)
    rospy.on_shutdown(node_shutdown)

    # Create a class instance
    ic = imageConverter()

    print("\nVideo recorder is waiting for the initialization...\n")

    while not rospy.is_shutdown():
      pass
    
    ic.video_writer.release()
    cv2.destroyAllWindows()

    