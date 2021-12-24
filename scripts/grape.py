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
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

from ros_assignment.cfg import HsvConfig

class image_converter:
    def __init__(self):

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/depth_registered/image_rect")
        self.camera_info = rospy.wait_for_message("/thorvald_001/kinect2_front_camera/hd/camera_info", CameraInfo)
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.purple_mask_pub = rospy.Publisher("~purple_mask", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug", Image, queue_size=1)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.marker_id = 0
        self.markers = []

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

    def marker(self, frame_id, radius, color, pose):

        now = rospy.Time.now()

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = now
        #marker.header.seq = 0
        marker.type = 2
        marker.id = self.marker_id
        marker.lifetime = rospy.Duration(10.2)
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

        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = now
        #text_marker.header.seq = self.marker_seq
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
        
        #self.marker_seq += 1

        # Transform Pose
        self.tf_listener.waitForTransform("map", frame_id, rospy.Time.now(), rospy.Duration(2.0))
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

    def image_callback(self, rgb_data, depth_data):
        now = time.time()
        # imsgmsg to cv2
        cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")

        # depthmsg to cv2
        cv_depth = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")

        # Filter out anything but purple
        purple_mask = self.colour_filter(cv_image)

        # Preprocessing
        kernel_close = np.ones((15,15), dtype=np.uint8)
        mask = cv2.morphologyEx(purple_mask, cv2.MORPH_CLOSE, kernel_close)
        
        kernel_open = np.ones((15,15), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

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
                
                # Segment image
                cv2.drawContours(mask, [c], 0, color, -1)
                ix, iy = np.where(np.all(mask == color, axis=-1))
                
                # Find depth
                depth = np.nanmedian(cv_depth[cY,cX])
                
                try:
                    # Get bounding box
                    x0, x1 = min(ix), max(ix)
                    y0, y1 = min(iy), max(iy)
                    # Ignore grapes that they're too far
                    if depth < 3.0:
                    #if ((y1 - y0) * (x1 - x0) < 0.5e5) and (depths[n] < 3.0):
                        # Debug
                        cv2.rectangle(mask, (y0, x0), (y1, x1), color, thickness=3)
                    
                        # Find pose
                        real_p1 = self.camera_model.projectPixelTo3dRay((x0, y0))
                        real_p2 = self.camera_model.projectPixelTo3dRay((x1, y1))
                        real_point = np.asarray(self.camera_model.projectPixelTo3dRay((cX,cY)))
                        width = depth * abs(real_p1[0] - real_p2[0])
                        height = depth * abs(real_p1[1] - real_p2[1])
                        radius = 0.15#max(width, height)
                        pose_vector = (1. / real_point[2]) * real_point
                        pose = pose_vector * depth
                        marker, text_marker = self.marker(rgb_data.header.frame_id, radius, color, pose)
                    
                        # Check duplicates
                        if not self.is_duplicate(marker):
                            self.marker_id += 1
                            self.markers.append(marker)
                            self.markers.append(text_marker)
                
                        centers.append((cX, cY))
                        colors.append(color)
                        depths.append(depth)
                
                        # Draw centroids and bounding boxes
                        cv2.circle(mask, (cX,cY), 5, (0,255,0), -1)
                        cv2.rectangle(mask, (y0, x0), (y1, x1), color, thickness=3)
                except:
                    continue
                
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        self.marker_pub.publish(marker_array)
        
        # cv2 to imgmsg
        mask_msg = self.bridge.cv2_to_imgmsg(purple_mask, "mono8")
        mask_msg.header = rgb_data.header

        result_msg = self.bridge.cv2_to_imgmsg(mask, "bgr8")
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

