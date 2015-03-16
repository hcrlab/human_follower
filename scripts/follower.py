#!/usr/bin/env python

import roslib
import rospy
import actionlib

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def callback(data):
    # publish to whatever message the driving is going to take place
    pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #client.wait_for_server()
    
    if len(data.people) > 0:
        #person found
        rospy.loginfo("Found person, generate goal")
        target_goal_simple = PoseStamped()
        #target_goal = MoveBaseGoal()
        
        #forming a proper PoseStamped message
        target_goal_simple.pose.position = data.people[0].pos
        target_goal_simple.pose.position.z = 0
        target_goal_simple.pose.orientation.w = 1
        target_goal_simple.header.frame_id = 'map'
        target_goal_simple.header.stamp = rospy.Time.now()
        #target_goal.target_pose.pose.position = data.people[0].pos

        #sending goal
        rospy.loginfo("sending goal")
        pub.publish(target_goal_simple)
        #client.send_goal(target_goal)
    
def follower():
    rospy.init_node('human_follower')
    rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass

    
