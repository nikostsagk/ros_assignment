#! /usr/bin/env python

import rospy
import actionlib
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String, Time
from ros_assignment.srv import getGrapes

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class topoNavClient:

    def __init__(self):
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()

        # Init
        self.rgb_image = None
        self.depth_image = None
        self.image_topic = rospy.get_param("~image_topic", "/thorvald_001/kinect2_front_camera/hd/image_color_rect")
        self.depth_topic = rospy.get_param("~depth_topic", "/ros_assignment/depth_registered/image_rect")
        subscribers = [
                message_filters.Subscriber(self.image_topic, Image),
                message_filters.Subscriber(self.depth_topic, Image),
                ]
        self.ts = message_filters.ApproximateTimeSynchronizer(subscribers, 1, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.image_callback)
    
        goals = [
                "r3-c0", 
                "r3-c1",
                "r3-c2", 
                "r3-c3", 
                "r3-c4", 
                "r3-c5", 
                "r3-c6", 
                "r3-c7", 
                "r3-c8", 
                "r3-c9", 
                "r3-c10", 
                "r3-c11", 
                "r3-c12",
                "r3-c13",
                "r3-c14",
                "r3-c15",
                "r3-c16",
                "r3-c17",
                "r3-c18",
                "r3-c19",
                "r3-c20"]

        self.main(goals)
    
    def image_callback(self, rgb_data, depth_data):
        self.rgb_image = rgb_data
        self.depth_image = depth_data

    def main(self,goals):
        for g in goals:
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

