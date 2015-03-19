#!/usr/bin/env python

import roslib
import rospy
import actionlib
import tf

import math

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# constants
DIST_FROM_PREVIOUS = 3
DIST_FROM_TARGET = .5
RELIABILITY_MIN = .5

class ListenerSingleton:
    created = False
    listener = None

    @staticmethod
    def new():
        if (ListenerSingleton.created):
            return ListenerSingleton.listener
        else:
            ListenerSingleton.created = True
            ListenerSingleton.listener = tf.TransformListener()
            rospy.loginfo("created new instance of listener")
            return ListenerSingleton.listener    

class HumanFollower:

    def __init__(self):
        previousPose = None
        listener = tf.TransformListener()
        rospy.init_node('human_follower')

    def callback(data):
        # publish to whatever message the driving is going to take place
        pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
        positionPub = rospy.Publisher("currentPosition", PoseStamped, queue_size = 10)
        
        if len(data.people) > 0:
            # selecting most probable person.
            rospy.loginfo("selecting most probable person")

            maxReliability = RELIABILITY_MIN
            personIndex = -1

            for i in range(len(data.people)):
                if (data.people[i].reliability > maxReliability):
                    personIndex = i
            
            if (personIndex != -1):
                rospy.loginfo("Found person, generating goal")
                try:
                    (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time())
                    rospy.loginfo("Transform obtained")

                    # This is where the target person's legs are
                    legPosition = data.people[i].pos
                    
                    rospy.loginfo("Computing target")
                    # computing target point that is .2 meters away 
                    differenceX = legPosition.x - trans[0]
                    differenceY = legPosition.y - trans[1]

                    # forming a message of current position for visualization
                    curr_Position = PoseStamped()
                    curr_Position.pose.position.x = trans[0]
                    curr_Position.pose.position.y = trans[1]
                    curr_Position.pose.position.z = 0
                    curr_Position.pose.orientation.x = rot[0]
                    curr_Position.pose.orientation.y = rot[1]
                    curr_Position.pose.orientation.z = rot[2]
                    curr_Position.pose.orientation.w = rot[3]
                    curr_Position.header.frame_id = 'map'
                    curr_Position.header.stamp = rospy.Time.now()
                    
                    # calculating target location
                    angle = math.atan2(differenceY, differenceX)
                    length = math.hypot(differenceX, differenceY)
                    
                    target_length = length - DIST_FROM_TARGET

                    targetX = target_length * math.cos(angle) + trans[0]
                    targetY = target_length * math.sin(angle) + trans[1]

                    # forming target goal
                    target_goal_simple = PoseStamped()
                    target_goal_simple.pose.position.x = targetX
                    target_goal_simple.pose.position.y = targetY
                    target_goal_simple.pose.position.z = 0
                    target_goal_simple.pose.orientation.w = 1
                    target_goal_simple.header.frame_id = 'map'
                    target_goal_simple.header.stamp = rospy.Time.now()

                    # sending goal
                    rospy.loginfo("sending goal")
                    pub.publish(target_goal_simple)
                    positionPub.publish(curr_Position)

                except Exception as expt:
                    print type(expt)
                    print expt.args
            

    def run():
        rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        hf = HumanFollower()
        hf.run
    except rospy.ROSInterruptException:
        pass
