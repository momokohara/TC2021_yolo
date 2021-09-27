#!/usr/bin/env python
# coding: utf-8


import rospy
import cv2
import math
import numpy as np
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes
from std_srvs.srv import Trigger


class ObjectTracker():

    def __init__(self):
        self._cv_bridge = CvBridge()
        self._captured_image = None
        self._object_pixels = 0  # Maximum area detected in the current image[pixel]
        self._object_pixels_default = 0  # Maximum area detected from the first image[pixel]
        self._point_of_centroid = None
        # self._pub_pbject_image = rospy.Publisher("object", Image, queue_size=1)
        # self._pub_cmdvel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    """
    def _output_image(self, msg):
        output_image = self._cv_bridge.imgmsg_to_cv2(msg,'bgr8')
        return output_image 
    """
    def callback(self, msg):
        try:
            # self._captured_image = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            #print(type(msg))
            #print(int(msg))
            box_xmin = 0
            box_xmax = 0
            box_ymin = 0
            box_ymax = 0
            width = 640
            height = 480
            probability = 0
            (x_center, y_center) = (0, 0)
 
            for box in msg.bounding_boxes:
                box_xmin = float(box.xmin)
                print("box_xmin : " + str(box_xmin))
                box_xmax = float(box.xmax)
                print("box_xmax : " + str(box_xmax))
                box_ymin = float(box.ymin)
                print("box_ymin : " + str(box_ymin))
                box_ymax = float(box.ymax)
                print("box_ymax : " + str(box_ymax))
                probability = box.probability
                (x_center, y_center) = ((box_xmin + box_xmax)//2, (box_ymin + box_ymax)//2)
                print("x_center : " + str(x_center))
                print("y_center : " + str(y_center))

        except CvBridgeError as e:
            rospy.logerr(e)
        # img = self._cv_bridge.cv2_to_imgmsg(self._output_image, 'bgr8')
        # self._pub_pbject_image.publish(img)

if __name__ == '__main__':
    rospy.init_node('object_tracking')
    ot = ObjectTracker()
    # rospy.Subscriber("/detections_image_topic", Image, ot._output_image, queue_size=1)
    rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, ot.callback, queue_size=1)
    rospy.spin()
 
