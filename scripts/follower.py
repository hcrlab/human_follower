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
DIST_FROM_PREVIOUS = .3 # how close is too close that robot won't send a new goal
DIST_FROM_TARGET = .6 # how far away the robot should stop from the target
PROXIMITY_MAX = .4 # how far from last known position leg detector should consider to be probable
R_SCALE = .5
RELIABILITY_MIN = .4 #minimum reliability of the position

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
        self.previousGoal = None
        self.lastKnownPosition = None
        self.trackedObjectID = "Steve"

    def callback(self,data):
        # publish to whatever message the driving is going to take place
        pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size = 10)
        positionPub = rospy.Publisher("currentPosition", PoseStamped, queue_size = 10)
        
        if len(data.people) > 0:
            # selecting most probable person
            rospy.loginfo("Looking for suitible target")

            maxReliability = RELIABILITY_MIN
            personIndex = -1

            for i in range(len(data.people)):
                
                # reliability metric is based on a combination of leg_detector results
                # and how far this current goal is from the pervious goal.
                # if the same person is still in sight, it is the most reliable
                # If there is no previous goal, then it's simply the leg_detector reliability

                
                if (self.previousGoal == None):
                    reliability = data.people[i].reliability
                else:
                    if (data.people[i].object_id == self.trackedObjectID):
                        reliability = 100
                    else:
                        currPersonPosition = data.people[i].pos

                        distFromLastKnownX = currPersonPosition.x - self.lastKnownPosition.pose.position.x
                        distFromLastKnownY = currPersonPosition.y - self.lastKnownPosition.pose.position.y
                        distFromLastKnown = math.hypot(distFromLastKnownX, distFromLastKnownY)

                        if (distFromLastKnown < PROXIMITY_MAX):
                            reliability = data.people[i].reliability + ((PROXIMITY_MAX - distFromLastKnown) * R_SCALE)
                        else:
                            reliability = data.people[i].reliability
                        
                if (reliability > maxReliability):
                    maxReliability = reliability
                    personIndex = i

            rospy.loginfo("count: " + str(len(data.people)))
            rospy.loginfo("final R: " + str(reliability))
            rospy.loginfo("max R: " + str(maxReliability))
                    
            # someone more probable than the mmin probability.
            if (personIndex != -1):
                rospy.loginfo("Target Found")
                try:
                    listener = ListenerSingleton.new()
                    (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
                    rospy.loginfo("Transform obtained")

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

                    # This is where the target person's legs are                                      
                    legPosition = data.people[personIndex].pos

                    rospy.loginfo("Computing goal")

                    # computing target point that is set distance away    
                    differenceX = legPosition.x - trans[0]
                    differenceY = legPosition.y - trans[1]
                    
                    # calculating target location
                    angle = math.atan2(differenceY, differenceX)
                    length = math.hypot(differenceX, differenceY)
                    
                    # calculating the position of the goal
                    target_length = length - DIST_FROM_TARGET
                    goalX = target_length * math.cos(angle) + trans[0]
                    goalY = target_length * math.sin(angle) + trans[1]

                    # calculating the orientation quaternion of the goal
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)


                    # forming target goal
                    target_goal_simple = PoseStamped()
                    
                    target_goal_simple.pose.position.x = goalX
                    target_goal_simple.pose.position.y = goalY
                    target_goal_simple.pose.position.z = 0
                    
                    target_goal_simple.pose.orientation.x = quaternion[0]
                    target_goal_simple.pose.orientation.y = quaternion[1]
                    target_goal_simple.pose.orientation.z = quaternion[2]
                    target_goal_simple.pose.orientation.w = quaternion[3]

                    target_goal_simple.header.frame_id = 'map'
                    target_goal_simple.header.stamp = rospy.Time.now()

                    # setting currently tracked objectID, last known position
                    self.trackedObjectID = data.people[personIndex].object_id
                    self.lastKnownPosition = target_goal_simple
                    
                    # sending goal
                    if (self.previousGoal == None):
                        rospy.loginfo("first goal woo hoo!")
                        self.previousGoal = target_goal_simple
                        rospy.loginfo("sending goal")
                        pub.publish(target_goal_simple)
                    else:
                        #calculating distance from previous goal
                        distX = target_goal_simple.pose.position.x - self.previousGoal.pose.position.x
                        distY = target_goal_simple.pose.position.y - self.previousGoal.pose.position.y
                        dist = math.hypot(distX, distY)
                        
                        if (dist > DIST_FROM_PREVIOUS):
                            self.previousGoal = target_goal_simple
                            rospy.loginfo("sending new goal")
                            pub.publish(target_goal_simple)
                        else:
                            rospy.loginfo("new goal canceled: too close to current goal!")

                    # publish current position for visualization
                    positionPub.publish(curr_Position)

                except Exception as expt:
                    print type(expt)
                    print expt.args
            

    def run(self):
        rospy.init_node("human_follower")
        rospy.Subscriber('people_tracker_measurements',PositionMeasurementArray, self.callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        hf = HumanFollower()
        hf.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("oh no, he's dead!")
