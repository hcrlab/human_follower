#!/usr/bin/env python

import roslib
import rospy
import actionlib

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def callback(data):
    # publish to whatever message the driving is going to take place
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #client.wait_for_server()
    
    if len(data.people) > 0:
        #person found
        rospy.loginfo("Found person, generate goal")
        target_goal = MoveBaseGoal()
        target_goal.target_pose.pose.position = data.people[0].pos
        
        #sending goal
        rospy.loginfo("sending goal")
        client.send_goal(target_goal)
    
def follower():
    rospy.init_node('human_follower')
    rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass

    
