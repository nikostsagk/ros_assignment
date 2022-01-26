#! /usr/bin/env python

import threading
import yaml
import rospy
import actionlib
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Time
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

        self.yaml_file = rospy.get_param("~nav_client_file", None)
        self.config = self.load_nav_file(self.yaml_file)

        self.client = {}
        
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
        for node in self.config[robot]["nodes"]:
            self.gotoNode(robot, node)
            for topics in zip(self.config[robot]["rgb_image_topics"],
                            self.config[robot]["depth_image_topics"],
                            self.config[robot]["camera_info_topics"]):
                self.getGrapes(topics)
    
    def gotoNode(self, robot, node):
        goal = GotoNodeGoal()
        goal.target = node
        self.client[robot].send_goal(goal)
        status = self.client[robot].wait_for_result()
        result = self.client[robot].get_result()
        rospy.loginfo("result for {:s} for {:s} is {:s}".format(robot, node, result))
    
    def getGrapes(self, topics):
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

    def load_nav_file(self, file):
        with open(file, 'r') as f:
            try:
                yaml_file = yaml.safe_load(f)
                return yaml_file
            except yaml.YAMLError as e:
                print(e)

    def main(self):
        for g in self.config["rows"]:

            # topo_nav client
            goal = GotoNodeGoal()
            goal.target = g
            self.client.send_goal(goal)
            status = self.client.wait_for_result() # wait until the action is complete
            result = self.client.get_result()
            rospy.loginfo("status for %s is %s", g, status)
            rospy.loginfo("result is %s", result)

            # getGrapes client
            rospy.wait_for_service("/ros_assignment/get_grapes")
            get_grapes = rospy.ServiceProxy("/ros_assignment/get_grapes", getGrapes)

            # wait for messages. Unfortunately ApproximateTimeSynchroniser can't wait for only one message
            rgb_image = rospy.wait_for_message(self.image_topic, Image)
            depth_image = rospy.wait_for_message(self.depth_topic, Image)
            camera_info = rospy.wait_for_message(self.camera_info, CameraInfo)

            frame_id = String()
            frame_id.data = rgb_image.header.frame_id
            timestamp = Time()
            timestamp.data = rgb_image.header.stamp
            resp = get_grapes(rgb_image, depth_image, camera_info, frame_id, timestamp)

            rospy.loginfo("{:s}: {:d} grapes\n".format(g, len(resp.coordinates.data)/3))


if __name__ == '__main__':
    try:
        rospy.init_node('topological_navigation_client')
        tnc = topoNavClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

