#!/usr/bin/env python

import cv2
import numpy as np
import random as rng
from image_geometry import PinholeCameraModel

import rospy
import ros_numpy
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
        """
            This node sets a server which returns detected grapes in an image frame.
            Besides the service, it aggregates all the detected grapes in msg/Marker[]
            and publishes each time there is a new detection.

            It's called by supplying:
                    (rgb msg/Image, depth msg/Image, camera_info msg/CameraInfo)
            and returns
                    (bboxes msg/Array[], xyz msg/Array[])

        """

        # Init
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()

        # Publishers
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        self.marker_id = 0
        self.markers = []

        # ROS Services
        rospy.Service("get_grapes", getGrapes, self.get_grapes)
        self.dyn_reconf_srv = Server(HsvConfig, self.dyn_reconf_callback)

    def dyn_reconf_callback(self, config, level):
        """
            HSV dynamic reconfigure callback
        """
        self.config = config
        for i in config.items():
          if hasattr(self, i[0]):
              setattr(self, i[0], i[1])
              print i[0], "set to:", getattr(self, i[0])
        return config

    def marker(self, stamp, frame_id, radius, color, pose):
        """
            Transforms grape xyz from camera coordinates to map
            and creates 2 msg/Marker one with the grape dimenions,
            and a one with its label. Both are appended in the
            same list.
        """

        # Create the grape marker
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.type = 2
        marker.id = self.marker_id
        marker.lifetime = rospy.Duration()
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

        # Create the label market
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = stamp
        text_marker.id = - self.marker_id - 1000
        text_marker.type = 9
        text_marker.text = str(self.marker_id)
        text_marker.pose.position.x = pose[0]
        text_marker.pose.position.y = pose[1] - marker.scale.y # offset it
        text_marker.pose.position.z = pose[2]
        text_marker.scale.z = marker.scale.z
        text_marker.color = marker.color
        text_marker.lifetime = marker.lifetime

        # Transform Pose from camera frame_id to map
        self.tf_listener.waitForTransform("map", frame_id, stamp, rospy.Duration(3.0))
        new_pose_m = self.tf_listener.transformPose("map", marker)
        new_pose_t = self.tf_listener.transformPose("map", text_marker)
        marker.pose = new_pose_m.pose
        text_marker.pose = new_pose_t.pose
        marker.header.frame_id = "map"
        text_marker.header.frame_id = "map"

        return marker, text_marker

    def is_duplicate(self, marker):
        """
            CAUTION:
                THIS METHOD IS SUPER STUPID, AND
                THE AUTHOR DESERVES A PAINFUL DEATH.

            Checks if there are any duplicate grapes.
            For each new detection, it iterates through the
            list with the aggregated markers and checks if
            the new grape is within olds' markers radius.

                ||xyz' - xyz|| <= (xyz)_radius

            where xyz is a marker point.

        """
        marker_pose = ros_numpy.numpify(marker.pose.position)
        for m in self.markers[::2]:
            mpose = ros_numpy.numpify(m.pose.position)
            if np.linalg.norm(marker_pose - mpose) <= 3*max(marker.scale.x, m.scale.x):
                return True
        return False

    def colour_filter(self, image):
        """
            Returns a filtered mask based on the HSV values
            specified in the rqt_reconfigure.
        """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_filter_r = np.array([self.config["H_min"], self.config["S_min"], self.config["V_min"]])
        upper_filter_r = np.array([self.config["H_max"], self.config["S_max"], self.config["V_max"]])
        mask = cv2.inRange(hsv_image, lower_filter_r, upper_filter_r)
        return mask

    def preprocessing(self, image):
        """
            Preprocessing pipeline:

            RGB -> HSV filtered mask -> Closing -> Opening
        """
        # Filter out anything but purple
        mask = self.colour_filter(image)

        # Preprocessing
        kernel = np.ones((15,15), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return mask

    def image_callback(self, rgb_img, depth_img, camera_model, frame_id, timestamp):
        """
            Postprocessing pipeline:

            Mask -> Segmentation (contours) -> Centroids -> xyz
        """
        mask = self.preprocessing(rgb_img)

        # Find contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.dstack([mask, mask, mask])

        centers = []
        colors = []
        depths = []
        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            # Filter out small areas (re-consider this, it does automatically the robot
            # distance dependent)
            if area > 500:
                # Append random color for each contour
                rng.seed(i)
                color = (rng.randint(0, 255), rng.randint(0,255), rng.randint(0,255))

                # Found centroids of contours
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])

                # Colorise each contour (essential step)
                cv2.drawContours(mask, [c], 0, color, -1)
                ix, iy = np.where(np.all(mask == color, axis=-1))

                # Find depth using known image coordinates
                depth = np.nanmedian(depth_img[cY,cX])

                try:
                #if True:
                    # Get bounding box
                    x0, x1 = min(ix), max(ix)
                    y0, y1 = min(iy), max(iy)
                    # Ignore grapes that they're too far (avoid detecting back rows)
                    if depth < 3.0:
                        # Get world coordinates from image coordinates
                        # real_p0: xyz of upper left bbox corner (for size estimation)
                        # real_p1: xyz of bottom right bbox corner
                        # real_pc: xyz of centroid
                        real_p0 = camera_model.projectPixelTo3dRay((x0, y0))
                        real_p1 = camera_model.projectPixelTo3dRay((x1, y1))
                        real_pC = np.asarray(camera_model.projectPixelTo3dRay((cX,cY)))
                        width = depth * abs(real_p0[0] - real_p1[0])
                        height = depth * abs(real_p0[1] - real_p1[1])
                        radius = max(width, height) # we assume they're circular
                        pose = (1. / real_pC[2]) * real_pC * depth

                        # Get markers
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

        # Aggregate the marker list and publish
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        self.marker_pub.publish(marker_array)
        return mask, centers

    def get_grapes(self, req):
        cv_rgb_img = self.bridge.imgmsg_to_cv2(req.rgb_image, "bgr8")
        cv_dep_img = self.bridge.imgmsg_to_cv2(req.depth_image, "32FC1")

        # Create PinholeCameraModel()
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(req.camera_info)

        mask, centers = self.image_callback(cv_rgb_img, cv_dep_img, camera_model, req.frame_id.data, req.timestamp.data)
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

