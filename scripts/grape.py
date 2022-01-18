#!/usr/bin/env python

import cv2
import numpy as np
import random as rng
from image_geometry import PinholeCameraModel

import rospy
import ros_numpy
import message_filters
import tf
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
import time
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

from ros_assignment.cfg import HsvConfig
from ros_assignment.srv import getGrapes, getGrapesResponse

class image_converter:
    def __init__(self):

        self.camera_info = rospy.wait_for_message(rospy.get_param("~camera_info"), CameraInfo)
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
       
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.marker_id = 0
        self.markers = []

        # ROS Services
        rospy.Service("get_grapes", getGrapes, self.get_grapes)
        self.dyn_reconf_srv = Server(HsvConfig, self.dyn_reconf_callback)

    def dyn_reconf_callback(self, config, level):
        self.config = config
        for i in config.items():
          if hasattr(self, i[0]):
              setattr(self, i[0], i[1])
              print i[0], "set to:", getattr(self, i[0])
        return config

    def marker(self, stamp, frame_id, radius, color, pose):

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.type = 2
        marker.id = self.marker_id
        marker.lifetime = rospy.Duration(10.0)
        marker.ns = "ros_assignment/grape_2d_segmentation/markers"
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius
        marker.color.b = color[0] / 255.
        marker.color.g = color[1] / 255.
        marker.color.r = color[2] / 255.
        marker.color.a = 1.0
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        marker.pose.orientation.w = 1.0

        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = stamp
        text_marker.ns = marker.ns
        text_marker.id = - self.marker_id - 1000
        text_marker.type = 9
        text_marker.text = str(self.marker_id)
        text_marker.pose.position.x = pose[0]
        text_marker.pose.position.y = pose[1] - marker.scale.y
        text_marker.pose.position.z = pose[2] 
        text_marker.scale.z = marker.scale.z
        text_marker.color = marker.color
        text_marker.lifetime = marker.lifetime
        
        # Transform Pose
        self.tf_listener.waitForTransform("map", frame_id, stamp, rospy.Duration(3.0))
        new_pose_m = self.tf_listener.transformPose("map", marker)
        new_pose_t = self.tf_listener.transformPose("map", text_marker)
        marker.pose = new_pose_m.pose
        text_marker.pose = new_pose_t.pose
        marker.header.frame_id = "map"
        text_marker.header.frame_id = "map"

        return marker, text_marker

    def is_duplicate(self, marker):
        marker_pose = ros_numpy.numpify(marker.pose.position)
        for m in self.markers[::2]:
            mpose = ros_numpy.numpify(m.pose.position)
            if np.linalg.norm(marker_pose - mpose) <= 3*max(marker.scale.x, m.scale.x):
                return True
        return False

    def colour_filter(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_filter_r = np.array([self.config["H_min"], self.config["S_min"], self.config["V_min"]])
        upper_filter_r = np.array([self.config["H_max"], self.config["S_max"], self.config["V_max"]])
        mask = cv2.inRange(hsv_image, lower_filter_r, upper_filter_r)
        return mask

    def preprocessing(self, image):
        # Filter out anything but purple
        purple_mask = self.colour_filter(image)

        # Preprocessing
        kernel_close = np.ones((15,15), dtype=np.uint8)
        mask = cv2.morphologyEx(purple_mask, cv2.MORPH_CLOSE, kernel_close)
        
        kernel_open = np.ones((15,15), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        return mask
    
    def image_callback(self, rgb_img, depth_img, frame_id, timestamp):
        mask = self.preprocessing(rgb_img)

        # Find contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        markers = np.zeros(mask.shape, dtype=np.int32)
        mask = np.dstack([mask, mask, mask])

        centers = []
        colors = []
        depths = []
        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            # Filter out small areas (re-think about this)
            if area > 500:
                # Append random color for each contour
                rng.seed(i)
                color = (rng.randint(0, 256), rng.randint(0,256), rng.randint(0,256))
                
                # Found centroids of contours
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                
                # Segment and colorise image
                cv2.drawContours(mask, [c], 0, color, -1)
                ix, iy = np.where(np.all(mask == color, axis=-1))
                
                # Find depth
                depth = np.nanmedian(depth_img[cY,cX])
                
                try:
                #if True:
                    # Get bounding box
                    x0, x1 = min(ix), max(ix)
                    y0, y1 = min(iy), max(iy)
                    # Ignore grapes that they're too far
                    if depth < 3.0:
                        # Find pose
                        real_p0 = self.camera_model.projectPixelTo3dRay((x0, y0))
                        real_p1 = self.camera_model.projectPixelTo3dRay((x1, y1))
                        real_pC = np.asarray(self.camera_model.projectPixelTo3dRay((cX,cY)))
                        width = depth * abs(real_p0[0] - real_p1[0])
                        height = depth * abs(real_p0[1] - real_p1[1])
                        radius = max(width, height)
                        pose = (1. / real_pC[2]) * real_pC * depth
                        marker, text_marker = self.marker(timestamp, frame_id, radius, color, pose)
                    
                        # Check duplicates
                        if not self.is_duplicate(marker):
                            self.marker_id += 1
                            self.markers.append(marker)
                            self.markers.append(text_marker)
                
                            centers.append((cX, cY))
                            colors.append(color)
                            depths.append(depth)
                
                except:
                    continue
                
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        self.marker_pub.publish(marker_array)
        return mask, centers

    def get_grapes(self, req):
        cv_rgb_img = self.bridge.imgmsg_to_cv2(req.rgb_image, "bgr8")
        cv_dep_img = self.bridge.imgmsg_to_cv2(req.depth_image, "32FC1")
        mask, centers = self.image_callback(cv_rgb_img, cv_dep_img, req.frame_id.data, req.timestamp.data)
        mask = self.bridge.cv2_to_imgmsg(mask, "bgr8")
        centers = Int16()
        centers.data = 1
        return getGrapesResponse(mask=mask, centers=centers)


def main():
    ic = image_converter()

if __name__ == "__main__":
    try:
        rospy.init_node('image_converter')
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

