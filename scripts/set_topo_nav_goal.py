#! /usr/bin/env python

import yaml
import rospy
import actionlib
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String, Time
from ros_assignment.srv import getGrapes

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class topoNavClient:

    def __init__(self):

        # Init
        self.rgb_image = None
        self.depth_image = None
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_color_rect")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect")

        print(self.image_topic, self.depth_topic)
        subscribers = [
                message_filters.Subscriber(self.image_topic, Image),
                message_filters.Subscriber(self.depth_topic, Image),
                ]
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 1, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.image_callback)

        self.yaml_file = rospy.get_param("~nav_client_file", None)
        self.config = self.load_nav_file(self.yaml_file)

        # Start topo_nav client
        self.client = actionlib.SimpleActionClient('/{:s}/topological_navigation'.format(self.config["robot"]), GotoNodeAction)
        self.client.wait_for_server()

        self.main()

    def image_callback(self, rgb_data, depth_data):
        self.rgb_image = rgb_data
        self.depth_image = depth_data

    def load_nav_file(self, file):
        with open(file, 'r') as f:
            try:
                yaml_file = yaml.safe_load(f)
                return yaml_file
            except yaml.YAMLError as e:
                print(e)

    def main(self):
        for g in self.config["rows"]:
            goal = GotoNodeGoal()
            goal.target = g
            self.client.send_goal(goal)
            status = self.client.wait_for_result() # wait until the action is complete
            result = self.client.get_result()
            rospy.loginfo("status for %s is %s", g, status)
            rospy.loginfo("result is %s", result)

            rospy.wait_for_service("/ros_assignment/get_grapes")
            get_grapes = rospy.ServiceProxy("/ros_assignment/get_grapes", getGrapes)
            rospy.sleep(1.0)
            frame_id = String()
            frame_id.data = self.rgb_image.header.frame_id
            timestamp = Time()
            timestamp.data = self.rgb_image.header.stamp
            resp = get_grapes(self.rgb_image, self.depth_image, frame_id, timestamp)


if __name__ == '__main__':
    try:
        rospy.init_node('topological_navigation_client')
        tnc = topoNavClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

