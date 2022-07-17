#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
# import the necessary packages

from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
import argparse
import imutils

from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox


def callback(msg):
    
    rospy.loginfo('received')
    image = bridge.imgmsg_to_cv2(msg,"bgr8")

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    image = imutils.resize(image, width=min(400, image.shape[1]))
    orig = image.copy()
    # cv2.imshow("msg",image)
    # detect people in the image
    (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
        padding=(8, 8), scale=1.05)
    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
    # draw the final bounding boxes
    bboxes = BoundingBoxes()
    bboxes.header.stamp = rospy.Time.now()
    bboxes.header.frame_id = "camera_link"
    bboxes.bounding_boxes = []
    bbox = BoundingBox()
    bbox.probability = 0
    bbox.xmin = -1
    bbox.ymin = -1
    bbox.xmax = -1
    bbox.ymax = -1
    bbox.Class = 'person'
    bboxes.bounding_boxes.append(bbox)
    for (xA, yA, xB, yB) in pick:
        # create msg
        bbox = BoundingBox()
        bbox.probability = 1
        bbox.xmin = xA
        bbox.ymin = yA
        bbox.xmax = xB
        bbox.ymax = yB
        bbox.Class = 'person'
        bboxes.bounding_boxes.append(bbox)
    pub.publish(bboxes)

    

    # show some information on the number of bounding boxes
    # show the output images
    # import pdb;pdb.set_trace()
    # cv2.imshow("Before NMS", orig)
    # cv2.namedWindow('After NMS', cv2.WINDOW_NORMAL)
    # cv2.imshow("After NMS", image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # cv2.imwrite("/home/cerlab/test/"+str(msg.header.stamp.secs) + "|" + str(msg.header.stamp.nsecs) +".jpg",image)



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    # cv2.namedWindow("Image Window", 1)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    # pub = rospy.Publisher('human_rect',BoundingBoxes)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/person_detector/human_rect',BoundingBoxes)
    bridge = CvBridge()
    listener()

# rospy.init_node('opencv_example', anonymous=True)
# # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
# rospy.loginfo("Hello ROS!")

# # Initialize the CvBridge class
# bridge = CvBridge()

# # Define a function to show the image in an OpenCV Window
# def show_image(img):
#     cv2.imshow("Image Window", img)
#     cv2.waitKey(3)

# # Define a callback for the Image message
# def image_callback(img_msg):
#     # log some info about the image topic
#     rospy.loginfo(img_msg.header)

#     # Try to convert the ROS Image message to a CV2 Image
#     # try:
#     cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#     # except CvBridgeError, e:
#     #     rospy.logerr("CvBridge Error: {0}".format(e))

#     # Show the converted image
#     show_image(cv_image)

# # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
# sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# # Initialize an OpenCV Window named "Image Window"
# cv2.namedWindow("Image Window", 1)

# # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
# while not rospy.is_shutdown():
#     rospy.spin()
