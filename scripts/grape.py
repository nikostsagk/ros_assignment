#!/usr/bin/env python

import os, errno
import cv2
import numpy as np
import random as rng
import datetime
from image_geometry import PinholeCameraModel

import rospy
import ros_numpy
import tf
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from std_msgs.msg import Int16MultiArray, String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from ros_assignment.cfg import HsvConfig
from ros_assignment.srv import getGrapes, getGrapesResponse

def mkdir(path):
    """
        Creates a directory if it does not already exists.

        Args
            path : path/to/new_directory (string)
    """
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise

class detectionService:
    def __init__(self):
        """
            This node sets a server which returns detected grapes in an image frame.
            Besides the service, it aggregates all the detected grapes in msg/Marker[]
            and publishes each time there is a new detection.

            It's called by supplying:
                    (rgb msg/Image, depth msg/Image, camera_info msg/CameraInfo)

            Returns an array of the centroids of the detected grapes, and their label
                [x0,y0,z0, x1,y1,z1, ..., xN,yN,zN]
                [l1, l2, l3, lN]

        """

        # Init
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.logging = rospy.get_param("~logging", True)
        self.marker_id = 0
        self.markers = []

        # Publishers
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, latch=True, queue_size=1)

        # ROS Services
        rospy.Service("get_grapes", getGrapes, self.get_grapes)
        self.dyn_reconf_srv = Server(HsvConfig, self.dyn_reconf_callback)

        # Logging dir
        if self.logging:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            output_dir = os.path.join(current_dir, os.pardir)
            output_dir = os.path.abspath(os.path.join(output_dir, "logs", str(datetime.date.today()).replace("-", "_")))
            self.logging_dir = output_dir
            mkdir(self.logging_dir)

    def dyn_reconf_callback(self, config, level):
        """
            HSV dynamic reconfigure callback
        """
        self.config = config
        for i in config.items():
          if hasattr(self, i[0]):
              setattr(self, i[0], i[1])
              print(i[0], "set to:", getattr(self, i[0]))
        return config

    def marker(self, stamp, frame_id, radius, color, pose, namespace):
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
        marker.ns = namespace
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
        text_marker.ns = marker.ns
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
        marker_pose = ros_numpy.numpify(marker.pose.position) # xyz of new marker
        for m in self.markers[::2]:
            mpose = ros_numpy.numpify(m.pose.position) # xyz of each stored marker
            if np.linalg.norm(marker_pose - mpose) <= max(marker.scale.x, m.scale.x): # scale = radius as markers are circles
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
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        return mask

    def image_callback(self, rgb_img, depth_img, camera_model, frame_id, timestamp):
        """
            Postprocessing pipeline:

            Mask -> Segmentation (contours) -> Centroids -> xyz
        """
        mask = self.preprocessing(rgb_img)

        robot_name, camera_name = frame_id.split("/")
        closest_node = rospy.wait_for_message("/{:s}/closest_node".format(robot_name), String).data
        namespace = "/{:s}/{:s}/{:s}".format(closest_node, robot_name, camera_name)

        # Find contours
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        coordinates = []
        colors = []
        depths = []
        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            # Filter out small areas (re-consider this, it does automatically the robot
            # distance dependent)
            if area > 100:
                # Append random color for each contour
                rng.seed(i)
                color = (rng.randint(0, 255), rng.randint(0,255), rng.randint(0,255))

                # Found centroids of contours
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])

                # Colorise each contour to find depth
                # (essential step to get all the depth values and choose the median)
                cimg = np.zeros_like(mask)
                cv2.drawContours(cimg, [c], 0, 255, -1)
                ix, iy = np.where(cimg == 255)
                depth = np.nanmedian(depth_img[ix, iy]) # ix,iy : the pixels of the segmented grape

                #try:
                if True:
                    # Get bounding box in camera coordinates
                    x0, y0, w, h = cv2.boundingRect(c)
                    x1 = x0 + w
                    y1 = y0 + h


                    # Ignore grapes that they're too far (avoid detecting back rows)
                    if depth < 3.0:
                        # Get world coordinates from image coordinates
                        # unit vector pointing the centroid
                        unit_vector = np.asarray(camera_model.projectPixelTo3dRay((cX,cY)))
                        width = camera_model.getDeltaX(w, depth)
                        height = camera_model.getDeltaY(h, depth)
                        radius = max(width, height) # we assume they're circular
                        pose = (1. / unit_vector[2]) * unit_vector * depth

                        # Logging
                        if self.logging:
                            path = "{:s}/{:s}/{:s}".format(self.logging_dir, closest_node, robot_name)
                            mkdir(path)
                            cv2.rectangle(rgb_img, (x0,y0), (x1,y1), color, 3)

                        # Get markers
                        marker, text_marker = self.marker(timestamp, frame_id, radius, color, pose, namespace)

                        # Check duplicates
                        if not self.is_duplicate(marker):
                            if self.logging:
                                rgb_img = cv2.putText(rgb_img, str(self.marker_id), (x1,y1+5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                            color, 2, cv2.FILLED)
                                cv2.imwrite("{:s}/{:s}.png".format(path, camera_name), rgb_img)
                                cv2.imwrite("{:s}/{:s}_binary.png".format(path, camera_name), mask)

                            self.marker_id += 1
                            self.markers.append(marker)
                            self.markers.append(text_marker)

                            coordinates.extend(pose) # points in camera coordinates = [x0,y0,z0,x1,y1,z1,...,xN,yN,zN]
                            colors.append(color)
                            depths.append(depth)

                # except:
                #     continue

        # Aggregate the marker list and publish
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        self.marker_pub.publish(marker_array)

        return mask, coordinates

    def get_grapes(self, req):
        """
            getGrapes service callback

            Sends an rgb msg/Image, a depth msg/Image,
            a PinholeCameraModel(), a frame_id, and a timestamp.

            Returns an array of the centroids of the detected grapes, and their label
                [x0,y0,z0, x1,y1,z1, ..., xN,yN,zN]
                [l1, l2, l3, lN]
        """
        cv_rgb_img = self.bridge.imgmsg_to_cv2(req.rgb_image, "bgr8")
        cv_dep_img = self.bridge.imgmsg_to_cv2(req.depth_image, "32FC1")

        # Create PinholeCameraModel()
        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(req.camera_info)

        # Get detections
        mask_, coordinates_ = self.image_callback(cv_rgb_img, cv_dep_img, camera_model, req.frame_id.data, req.timestamp.data)

        # Construct mask msg/Image and centers msg/Int16MultiArray
        mask = self.bridge.cv2_to_imgmsg(mask_, "mono8")
        coordinates = Int16MultiArray()
        coordinates.data = coordinates_

        return getGrapesResponse(mask=mask, coordinates=coordinates)


def main():
    ds = detectionService()

if __name__ == "__main__":
    try:
        rospy.init_node('detection_server')
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

