#!/usr/bin/env python

import roslib
import rospy
import actionlib
import tf

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# distances in meters
DIST_FROM_TARGET = .2

def callback(data):
    # publish to whatever message the driving is going to take place
    pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
    
    if len(data.people) > 0:
        #person found
        rospy.loginfo("Found person, generating goal")
        listener = tf.TransformListener()
        target_goal_simple = PoseStamped()
        
        #forming a proper PoseStamped message
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/map', rospy.Time.now())
            rospy.loginfo("Transform obtained x:" + trans[0] + " y:" + trans[1])

            # This is where the target person's legs are
            legPosition = data.people[0].pos
            
            rospy.loginfo("Computing target")
            # computing target point that is .2 meters away 
            difference = Point()
            difference.x = legPosition.x - trans[0]
            defference.y = legPosition.y - trans[1]
            
            angle = math.atan(difference.y, difference.y)
            length = sqrt(difference.x ** 2 + difference.y ** 2)
            
            target_length = length - DIST_FROM_TARGET

            targetX = target_length * math.cos(angle)
            targetY = target_length * math.sin(angle)

            target_goal_simple.pose.position.x = targetX
            target_goal_simple.pose.position.y = targetY
            target_goal_simple.pose.position.z = 0
            target_goal_simple.pose.orientation.w = 1
            target_goal_simple.header.frame_id = 'map'
            target_goal_simple.header.stamp = rospy.Time.now()

            # sending goal
            rospy.loginfo("sending goal")
            pub.publish(target_goal_simple)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("failed to get tf transform")
    
def follower():
    rospy.init_node('human_follower')
    rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass

    
