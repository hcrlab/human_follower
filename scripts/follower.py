#!/usr/bin/env python

import roslib
import rospy
import actionlib

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs import MoveBaseActionGoal

def callback(data):
    # publish to whatever message the driving is going to take place
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    
    if len(data.people) > 0:
        target_pose = MoveBaseGoal()
        target_pose.pose.position = data.people[0].pos

        client.send_goal(target_pose)
    
def follower():
    rospy.init_node('human_follower')
    rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass

    
