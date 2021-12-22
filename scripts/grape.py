#!/usr/bin/env python

import cv2
import numpy as np
import random as rng
from image_geometry import PinholeCameraModel
import rospy
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server

from ros_assignment.cfg import HsvConfig

class image_converter:
    def __init__(self):

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/depth_registered/image_rect")
        self.camera_info = rospy.wait_for_message("/thorvald_001/kinect2_right_camera/hd/camera_info", CameraInfo)
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.purple_mask_pub = rospy.Publisher("~purple_mask", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Image, queue_size=1)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.marker_seq = 0

        self.dyn_reconf_srv = Server(HsvConfig, self.dyn_reconf_callback)

        subscribers = [
                message_filters.Subscriber(self.image_topic, Image),
                message_filters.Subscriber(self.depth_topic, Image),
                ]
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 1, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.image_callback)

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

    def marker(self, frame_id, id, radius, color, pose):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.header.seq = self.marker_seq
        marker.type = 2
        marker.id = id
        marker.lifetime = rospy.Duration(0.2)
        #marker.text = str(id)
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

        self.marker_seq += 1

        return marker

    def image_callback(self, rgb_data, depth_data):
        # imsgmsg to cv2
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")

        # depthmsg to cv2
        cv_depth = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")

        # Filter out anything but purple
        purple_mask = self.colour_filter(cv_image)

        # Preprocessing
        kernel_close = np.ones((17,17), dtype=np.uint8)
        mask = cv2.morphologyEx(purple_mask, cv2.MORPH_CLOSE, kernel_close)
        
        kernel_open = np.ones((17,17), dtype=np.uint8)
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
                rng.seed(i)
                color = (rng.randint(0, 256), rng.randint(0,256), rng.randint(0,256))
                
                centers.append((cX, cY))
                colors.append(color)

        # mask to RGB
        result = np.dstack([mask, mask, mask])

        # Assign id to each contour and colorise it
        cv2.watershed(result, markers)
        labels = np.unique(markers)
        depths = []
        for n, label in enumerate(labels[1::]):
            ix, iy = np.where(markers == label)
            result[ix,iy,:] = colors[n]

            # Find depth values
            depth = np.nanmedian(cv_depth[ix,iy])
            depths.append(depth)

        #result = cv2.bitwise_and(mask_rgb, markers)
        
        # Draw bounding boxes around contours
        bboxes = []
        markers = []
        for n, color in enumerate(colors):
            ix, iy = np.where(np.all(result == color, axis=-1))
            if True:
                x0, x1 = min(ix), max(ix)
                y0, y1 = min(iy), max(iy)
                # Ignore misslocalisations and grapes from other rows
                if ((y1 - y0) * (x1 - x0) < 0.5e5) and (depths[n] < 3.0):
                    # Debug
                    cv2.rectangle(cv_image, (y0, x0), (y1, x1), color, thickness=3)
                    cv2.circle(cv_image, centers[n], 10, color, -1)
                    # Find pose
                    real_p1 = self.camera_model.projectPixelTo3dRay((x0, y0))
                    real_p2 = self.camera_model.projectPixelTo3dRay((x1, y1))
                    real_point = np.asarray(self.camera_model.projectPixelTo3dRay(centers[n]))
                    width = depths[n] * abs(real_p1[0] - real_p2[0])
                    height = depths[n] * abs(real_p1[1] - real_p2[1])
                    radius = max(width, height)
                    pose_vector = (1. / real_point[2]) * real_point
                    pose = pose_vector * depths[n]
                    marker = self.marker(rgb_data.header.frame_id, n, radius, color, pose)
                    markers.append(marker)
            #except:
            #    continue

        marker_array = MarkerArray()
        marker_array.markers = markers
        self.marker_pub.publish(marker_array)
        
        # cv2 to imgmsg
        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        mask_msg.header = rgb_data.header

        result_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        result_msg.header = rgb_data.header

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

