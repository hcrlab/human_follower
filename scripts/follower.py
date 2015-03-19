#!/usr/bin/env python

import roslib
import rospy
import actionlib
import tf

import math

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# distances in meters
DIST_FROM_PREVIOUS = 3
DIST_FROM_TARGET = .5

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

    def callback(data):
        # publish to whatever message the driving is going to take place
        pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
    positionPub = rospy.Publisher("currentPosition", PoseStamped, queue_size = 10)
    
    if len(data.people) > 0:
        #person found
        rospy.loginfo("Found person, generating goal")

        # selecting most probable person.
        rospy.loginfo("selecting most probagly person")
        maxReliability = 0
        personIndex = 0
        for i in range(len(data.people)):
            if (data.people[i].reliability > maxReliability):
                personIndex = i

        listener = ListenerSingleton.new()
        target_goal_simple = PoseStamped()
        
        #forming a proper PoseStamped message
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
            rospy.loginfo("Transform obtained")

            # This is where the target person's legs are
            legPosition = data.people[i].pos
            
            rospy.loginfo("Computing target")
            # computing target point that is .2 meters away 
            differenceX = legPosition.x - trans[0]
            differenceY = legPosition.y - trans[1]

            # publishing current position for visualization
            curr_location = PoseStamped()
            curr_location.pose.position.x = trans[0]
            curr_location.pose.position.y = trans[1]
            curr_location.pose.position.z = 0
            curr_location.pose.orientation.x = rot[0]
            curr_location.pose.orientation.y = rot[1]
            curr_location.pose.orientation.z = rot[2]
            curr_location.pose.orientation.w = rot[3]
            curr_location.header.frame_id = 'map'
            curr_location.header.stamp = rospy.Time.now()
            
            # calculating target location
            angle = math.atan2(differenceY, differenceX)
            length = math.hypot(differenceX, differenceY)
            
            target_length = length - DIST_FROM_TARGET

            targetX = target_length * math.cos(angle) + trans[0]
            targetY = target_length * math.sin(angle) + trans[1]

            # forming target goal
            target_goal_simple.pose.position.x = targetX
            target_goal_simple.pose.position.y = targetY
            target_goal_simple.pose.position.z = 0
            target_goal_simple.pose.orientation.w = 1
            target_goal_simple.header.frame_id = 'map'
            target_goal_simple.header.stamp = rospy.Time.now()

            # sending goal
            rospy.loginfo("sending goal")
            pub.publish(target_goal_simple)
            positionPub.publish(curr_location)

        except Exception as expt:
            print type(expt)
            print expt.args
    

def follower():
    rospy.init_node('human_follower')
    rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass
