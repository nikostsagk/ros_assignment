#!/usr/bin/env python

import cv2
import numpy as np
import random as rng
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
                                          Image, self.image_callback)
        self.purple_mask_pub = rospy.Publisher("/purple_mask", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("/debug", Image, queue_size=1)

    def image_callback(self, data):
        # imsgmsg to cv2
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Covnert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out anything but purple
        purple_mask = cv2.inRange(hsv_image, (100, 20, 20), (165, 255, 255))

        # Debug
        kernel = np.ones((3,3), dtype=np.uint8)
        debug = cv2.morphologyEx(purple_mask, cv2.MORPH_CLOSE, kernel)
        #debug = cv2.morphologyEx(debug, cv2.MORPH_CLOSE, kernel)

        #dist = cv2.distanceTransform(debug, cv2.DIST_L2, 3)
        #cv2.normalize(dist, dist, 0, 1.0, cv2.NORM_MINMAX)
        #_, dist = cv2.threshold(dist, 0.4, 1.0, cv2.THRESH_BINARY)
        #kernel = np.ones((3,3), dtype=np.uint8)
        #dist = cv2.dilate(dist, kernel)
        #dist_8u = dist.astype("uint8")


        #_, contours, _ = cv2.findContours(dist_8u, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #markers = np.zeros(dist.shape, dtype=np.int32)
        #for i in range(len(contours)):
        #    cv2.drawContours(markers, contours, i, (i+1), -1)
        #cv2.circle(markers, (5,5), 1, (255,255,255), -1)
        #markers_8u = (markers*10).astype("uint8")

        #imgResult = np.dstack([purple_mask, purple_mask, purple_mask])
        #cv2.watershed(imgResult, markers)

        #colors = []
        #for contour in contours:
        #    colors.append((rng.randint(0, 256), rng.randint(0,256), rng.randint(0,256)))

        #dst = np.zeros((markers.shape[0], markers.shape[1], 3), dtype=np.uint8)
        #for i in range(markers.shape[0]):
        #    for j in range(markers.shape[1]):
        #        index = markers[i,j]
        #        if index > 0 and index <= len(contours):
        #            dst[i,j,:] = colors[index-1]

        #dst = cv2.bitwise_and(dst, np.dstack([purple_mask, purple_mask, purple_mask]))

        # cv2 to imgmsg
        purple_mask_msg = self.bridge.cv2_to_imgmsg(purple_mask, "mono8")
        purple_mask_msg.header = data.header

        debug_msg = self.bridge.cv2_to_imgmsg(debug, "mono8")
        debug_msg.header = data.header

        # publish masks
        self.purple_mask_pub.publish(purple_mask_msg)
        self.debug_pub.publish(debug_msg)

def main():
    ic = image_converter()

if __name__ == "__main__":
    try:
        rospy.init_node('image_converter')
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

