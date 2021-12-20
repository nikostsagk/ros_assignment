#!/usr/bin/env python

import cv2
import numpy as np
import random as rng
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server

from ros_assignment.cfg import HsvConfig

class image_converter:
    def __init__(self):

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.purple_mask_pub = rospy.Publisher("~purple_mask", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Image, queue_size=1)

        self.dyn_reconf_srv = Server(HsvConfig, self.dyn_reconf_callback)

    def dyn_reconf_callback(self, config, level):
        self.config = config
        for i in config.items():
          if hasattr(self, i[0]):
              setattr(self, i[0], i[1])
              print i[0], "set to:", getattr(self, i[0])
        return config

    def colour_filter(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_filter_r = np.array([self.config["H_min"], self.config["S_min"], self.config["V_min"]])
        upper_filter_r = np.array([self.config["H_max"], self.config["S_max"], self.config["V_max"]])
        mask = cv2.inRange(hsv_image, lower_filter_r, upper_filter_r)
        return mask

    def image_callback(self, data):
        # imsgmsg to cv2
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Filter out anything but purple
        purple_mask = self.colour_filter(cv_image)

        # Preprocessing
        kernel_close = np.ones((9,9), dtype=np.uint8)
        mask = cv2.morphologyEx(purple_mask, cv2.MORPH_CLOSE, kernel_close)
        
        kernel_open = np.ones((13,13), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        # Find contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        markers = np.zeros(mask.shape, dtype=np.int32)

        centers = []
        colors = []
        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            # Filter out small areas (re-think about this)
            if area > 500:
                # Found centroids of contours
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                
                # Draw centroids
                cv2.circle(markers, (cX,cY), 10, (i+1), -1)
                
                # Append random color for each contour
                color = (rng.randint(0, 256), rng.randint(0,256), rng.randint(0,256))
                
                centers.append((cX, cY))
                colors.append(color)

        # mask to RGB
        result = np.dstack([mask, mask, mask])

        # Assign id to each contour and colorise it
        cv2.watershed(result, markers)
        labels = np.unique(markers)
        for n, label in enumerate(labels[1::]):
            ix, iy = np.where(markers == label)
            result[ix,iy,:] = colors[n]

        #result = cv2.bitwise_and(mask_rgb, markers)
        
        # Draw bounding boxes around contours
        bboxes = []
        for n, color in enumerate(colors):
            ix, iy = np.where(np.all(result == color, axis=-1))
            try:
                x0, x1 = min(ix), max(ix)
                y0, y1 = min(iy), max(iy)
                if (y1 - y0) * (x1 - x0) < 0.5e5:
                    cv2.rectangle(cv_image, (y0, x0), (y1, x1), color, thickness=3)
            except:
                continue

        # cv2 to imgmsg
        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        mask_msg.header = data.header

        result_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        result_msg.header = data.header

        # publish masks
        self.purple_mask_pub.publish(mask_msg)
        self.debug_pub.publish(result_msg)

def main():
    ic = image_converter()

if __name__ == "__main__":
    try:
        rospy.init_node('image_converter')
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

