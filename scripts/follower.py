#!/usr/bin/env python

import roslib
import rospy
import tf

import math
import sys, os

from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# constants
DIST_MIN = .3 # how close is too close that robot won't send a new goal
DIST_MAX = 3 # how far is too far that robot should not consider it as new person
ANGLE_THRESHOLD = math.pi / 6 # how wide is too wide robot will send new goal
DIST_FROM_TARGET = .5 # how far away the robot should stop from the target
PROXIMITY_MAX = .4 # how far from last known position leg detector should consider to be probable
R_SCALE = .5 # scale from distance to reliability
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

class GoalEuler:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

class HumanFollower:

    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size = 10)
        self.positionPub = rospy.Publisher("currentPosition", PoseStamped, queue_size = 10)

        self.lastKnownPosition = None
        self.trackedObjectID = "Steve"


    def callback(self, data):
        # get transform
        listener = ListenerSingleton.new()
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time())
        rospy.loginfo("Transform obtained")

        # sends current position for visualization
        self.sendCurrentPosition(trans, rot)

        # process leg detector input
        if len(data.people) > 0:
            personIndex = self.findReliableTarget(data, trans)

            # found someone more probable than the min probability.
            if (personIndex != -1):
                rospy.loginfo("Target Found")
                
                try:

                    # logs the start of goal computation
                    rospy.loginfo("Computing goal")

                    # This is where the target person's legs are                                      
                    legPosition = data.people[personIndex].pos

                    # setting last known position regardless of if the goal is sent or not
                    # angle is not important. Last Known position only needs the coordinates
                    self.lastKnownPosition = GoalEuler(legPosition.x, legPosition.y, 0)                    

                    # computing target point that is set distance away    
                    differenceX = legPosition.x - trans[0]
                    differenceY = legPosition.y - trans[1]
                    
                    # calculating target location
                    goalAngle = math.atan2(differenceY, differenceX)
                    length = math.hypot(differenceX, differenceY)
                    
                    # calculating the position of the goal
                    target_length = length - DIST_FROM_TARGET
                    goalX = target_length * math.cos(goalAngle) + trans[0]
                    goalY = target_length * math.sin(goalAngle) + trans[1]

                    # publish computed goal
                    (xErr, yErr, angleErr) = self.getError(goalX, goalY, goalAngle, trans, rot)

                    ## make twist messages
                    cmd = Twist()
                    cmd.linear.x = math.hypot(xErr, yErr)
                    cmd.angular.z = angleErr

                    rospy.loginfo("sending twist message")
                    rospy.loginfo("linear x:" + str(cmd.linear.x))
                    rospy.loginfo("angular z:" + str(cmd.angular.z))
                    self.pub.publish(cmd)

                except Exception as expt:
                    #exc_type, exc_obj, exc_tb = sys.exc_info()
                    #fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
                    #print(exc_type, fname, exc_tb.tb_lineno)
                    #print type(expt)
                    print expt.args
                

    def getError(self, goalX, goalY, angle, trans, rot):
        # calculates the error in angle between current pose and goal vector
        # assumes inputs in the same frame

        q = (rot[0], rot[1], rot[2], rot[3])
        self_angles = tf.transformations.euler_from_quaternion(q)

        angleErr = angle - self_angles[2] # rotation around z axis
        xErr = goalX - trans[0]
        yErr = goalY - trans[1]

        return (xErr, yErr, angleErr)

            
    def findReliableTarget(self, data, roboPosition):
        # selecting most probable person
        rospy.loginfo("Filtering for suitible target")

        maxReliability = RELIABILITY_MIN
        reliability = 0
        personIndex = -1

        for i in range(len(data.people)):
            
            # reliability metric is based on a combination of leg_detector results
            # and how far this current goal is from the pervious goal.
            # if the same person is still in sight, it is the most reliable
            # If there is no previous goal, then it's simply the leg_detector reliability

            currPersonPosition = data.people[i].pos
            distFromRobot = math.hypot(currPersonPosition.x - roboPosition[0], currPersonPosition.y - roboPosition[1])

            if (data.people[i].object_id == self.trackedObjectID):
                reliability = 100
            elif (distFromRobot > DIST_MAX):
                reliability = -100
            elif (self.lastKnownPosition != None):
                distFromLastX = currPersonPosition.x - self.lastKnownPosition.x
                distFromLastY = currPersonPosition.y - self.lastKnownPosition.y
                distFromLastKnown = math.hypot(distFromLastX, distFromLastY)

                # general case not the first goal
                if (distFromLastKnown < PROXIMITY_MAX):
                    reliability = data.people[i].reliability + ((PROXIMITY_MAX - distFromLastKnown) * R_SCALE)
                else:
                    reliability = data.people[i].reliability
            else:
                reliability = data.people[i].reliability
     
            if (reliability > maxReliability):
                maxReliability = reliability
                personIndex = i

        rospy.loginfo("count: " + str(len(data.people)))
        rospy.loginfo("final R: " + str(reliability))

        return personIndex


    def sendCurrentPosition(self, trans, rot):
        curr_Position = PoseStamped()
        curr_Position.pose.position.x = trans[0]
        curr_Position.pose.position.y = trans[0]
        curr_Position.pose.position.z = 0
        curr_Position.pose.orientation.x = rot[0]
        curr_Position.pose.orientation.y = rot[1]
        curr_Position.pose.orientation.z = rot[2]
        curr_Position.pose.orientation.w = rot[3]
        curr_Position.header.frame_id = 'map'
        curr_Position.header.stamp = rospy.Time.now()

        # publishing current position for visualization
        self.positionPub.publish(curr_Position)

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
