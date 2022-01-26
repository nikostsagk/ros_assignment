#! /usr/bin/env python

import threading
import yaml
import rospy
import actionlib
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Time
from visualization_msgs.msg import MarkerArray
from ros_assignment.srv import getGrapes

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class topoNavClient:

    def __init__(self):
        """
            Initiates an ActionClient and sends goals to the topological_navigation
            In each node, grabs a frame, and calls the getGrapes service.

            Publishes:
                 the resulting image with the drawn bounding boxes
                 the binary HSV mask (for debugging purposes)
        """

        # Init
        self.yaml_file = rospy.get_param("~nav_client_file", None)
        self.config = self.load_nav_file(self.yaml_file)

        self.client = {}

        # Organise published markers by row
        self.nodes = set()
        for robot in self.config.keys():
            for node in self.config[robot]["nodes"]:
                self.nodes.add(node.split("-")[0]) # e.g. (r0, r1, ..., rN)

        # Subs
        self.marker_list = {row : [] for row in self.nodes}
        self.marker_topic = rospy.get_param("~marker_topic", "/ros_assignment/grape_2d_segmentation/markers")
        self.marker_sub = rospy.Subscriber(self.marker_topic, MarkerArray, callback=self.marker_cb, queue_size=1)

        # Start
        for robot in self.config.keys():
            # Start topo_nav clients
            self.client[robot] = actionlib.SimpleActionClient('/{:s}/topological_navigation'.format(robot), GotoNodeAction)
            self.client[robot].wait_for_server()
            rospy.loginfo("/{:s}/topological_navigation is ready".format(robot))

        threads = list()
        for robot in self.config.keys():
            x = threading.Thread(target=self.start, args=(robot,))
            threads.append(x)
            x.start()

    def start(self, robot):
        """
            gotoNode and then getGrapes
        """
        for node in self.config[robot]["nodes"]:
            self.gotoNode(robot, node)
            for topics in zip(self.config[robot]["rgb_image_topics"],
                            self.config[robot]["depth_image_topics"],
                            self.config[robot]["camera_info_topics"]):
                self.getGrapes(topics)
        # Assuming the robot has finished
        self.gotoNode(robot, self.config[robot]["on_end"][0])

    def gotoNode(self, robot, node):
        """
            Sends a robot to a specific node
        """
        goal = GotoNodeGoal()
        goal.target = node
        self.client[robot].send_goal(goal)
        status = self.client[robot].wait_for_result()
        result = self.client[robot].get_result()
        rospy.loginfo("result for {:s} for {:s} is {:s}".format(robot, node, result))

    def getGrapes(self, topics):
        """
            Calls the getGrapes service
        """
        rospy.wait_for_service("/ros_assignment/get_grapes")
        get_grapes = rospy.ServiceProxy("/ros_assignment/get_grapes", getGrapes)

        rgb_image = rospy.wait_for_message(topics[0], Image)
        depth_image = rospy.wait_for_message(topics[1], Image)
        camera_info = rospy.wait_for_message(topics[2], CameraInfo)

        frame_id = String()
        frame_id.data = rgb_image.header.frame_id
        timestamp = Time()
        timestamp.data = rospy.Time.now()#rgb_image.header.stamp
        resp = get_grapes(rgb_image, depth_image, camera_info, frame_id, timestamp)

        # markers = rospy.wait_for_message(self.marker_topic, MarkerArray)
        # self.marker_cb(markers)

    def marker_cb(self, marker_array):
        for marker in marker_array.markers:
            for row in self.nodes:
                if (row in marker.ns) and (marker not in self.marker_list[row]):
                    self.marker_list[row].append(marker)
        
        for row in self.nodes:
            rospy.loginfo("{:s}: {:d} grapes".format(row, len(self.marker_list[row])))

    def load_nav_file(self, file):
        """
            Loads a yaml file
        """
        with open(file, 'r') as f:
            try:
                yaml_file = yaml.safe_load(f)
                return yaml_file
            except yaml.YAMLError as e:
                print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('topological_navigation_client')
        tnc = topoNavClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

