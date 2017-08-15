#!/usr/bin/env python
import time
import rospy

from std_srvs.srv import Empty

# if __name__ == '__main__':
time.sleep(5)
rospy.init_node('stitch_loop')

rospy.wait_for_service('/image_saver1/save')
rospy.wait_for_service('/image_saver2/save')
rospy.wait_for_service('/stitch')
rospy.wait_for_service('/reset')
time.sleep(5)
save_image1 = rospy.ServiceProxy('/image_saver1/save', Empty)
save_image2 = rospy.ServiceProxy('/image_saver2/save', Empty)
stitch = rospy.ServiceProxy('/stitch', Empty)
reset = rospy.ServiceProxy('/reset', Empty)
time.sleep(5)


while True:
  time.sleep(5)
  reset()
  rospy.loginfo("Saving image from camera #1")
  save_image1()
  rospy.loginfo("Saving image from camera #2")
  save_image2()
  rospy.loginfo("Stitching panorama...")
  time.sleep(1) # wait for image files to be written
  stitch()
  rospy.loginfo("Done. Looping...")
