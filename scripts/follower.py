import rospy
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs import *

def callback(data):
    # publish to whatever message the driving is going to take place
    pub = rospy.Publisher('move_base_simple/goal', 'PoseStamped', queue_size=10)
    
    target_pose = PoseStamped()
    target_pose.pose.position = data.people[0].pos

    pub.publish(target_pose)
    
def follower():
    rospy.init_node('human_follower')
    rospy.Subscriber('people_tracker_measurements','PositionMeasurementArray', callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower()
    except rospy.ROSInterruptException:
        pass

    
