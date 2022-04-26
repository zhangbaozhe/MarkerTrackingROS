#!/usr/bin/env python

"""
Author: Baozhe Zhang
Date: April 25, 2022
Usage: ECE4310 Project 1
"""

import rospy 
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker # Marker tag
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
import tf, tf2_ros, tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from math import pi, acos, sqrt


##########################################################################################
# Global constants
##########################################################################################
## id(int) : (lower bound, upper bound)(tuple)
ENCODED_DIRS = {0: (0, pi/4), 1: (pi/4, pi/2), 2: (pi/2, 3*pi/4), 3: (3*pi/4, pi), 
                4: (-pi, -3*pi/4), 5: (-3*pi/4, -pi/2), 6: (-pi/2, -pi/4), 7: (-pi/4, 0), 
                8: None}
## initial id
INIT_ID = 4
## minimum stop distance (m)
MIN_STOP_DIS = 2.5
## target frame id
# TARGET_FRAME_ID = "map"
TARGET_FRAME_ID = "odom"
##########################################################################################




##########################################################################################
# Global variables
##########################################################################################
## should the navigation to be stopped?
IS_STOP = False
## is the robot on the way to navigate to the target? If true, we do not accept new target 
## otherwise, we scan for the new target 
IS_ON_NAV_ROAD = False
IS_ON_EXPLORE = False
## IMPORTANT!!! REMEBER TO UPDATE THE THREE BELOW AT THE SAME TIME
## This is the global target sequence containing the PoseStamped (in the odom or map coordinate) with the last of it as the new target
TARGET_SEQUENCE = []
## This is the global target ID sequence
TARGET_ID_SEQUENCE = []

MOVE_BASE_STATUS = 0
##########################################################################################



def initTarget(marker):
    """This function initializes the target sequences

    Args:
        marker (AlvarMarker):
    """
    global TARGET_ID_SEQUENCE, TARGET_SEQUENCE, IS_ON_NAV_ROAD
    print("[initTarget] marker.id =", marker.id)
    if marker.id == INIT_ID:
        TARGET_ID_SEQUENCE.append(INIT_ID)
        # TODO: how to use the best transformed pose? average?
        TARGET_SEQUENCE.append(fromCameraToOdom(marker))
        IS_ON_NAV_ROAD = True

def getTargetMarker(markersInCamera):
    """ MAIN CALLBACK FUNCTION
    This function receives the detected markers in the camera frame and add
    new target in the target sequence

    Args:
        markersInCamera (AlvarMarkers): The markers in the virtual_camera_link frame
    """
    global TARGET_SEQUENCE, TARGET_ID_SEQUENCE, IS_ON_NAV_ROAD, IS_ON_EXPLORE
    print("[getTargetMarker] TARGET_ID_SEQUENCE", TARGET_ID_SEQUENCE)
    # init process
    if len(TARGET_ID_SEQUENCE) == 0:
        # may be buggy, but let's try first
        print("[getTargetMarker] len(markersInCamera.markers) =", len(markersInCamera.markers))
        if len(markersInCamera.markers) == 1:
            initTarget(markersInCamera.markers[0])

    # main loop for adding the new target
    for marker in markersInCamera.markers:
        if marker.id == TARGET_ID_SEQUENCE[-1]:
            if isApproachOK(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z):
                print("[getTargetMarker] IS_ON_NAV_ROAD =", IS_ON_NAV_ROAD)
                IS_ON_NAV_ROAD = False
                IS_ON_EXPLORE = True
                break
        if IS_ON_NAV_ROAD:
            # we have not finish last nav yet
            break
        if marker.id in TARGET_ID_SEQUENCE:
            # we find the old one
            continue
        # this pose is ready to be added, a candidate target
        poseInOdom = fromCameraToOdom(marker)
        # if the checking process of the candidate pose is OK 
        # then we put the PoseStamped in the SEQUENCE
        if isPoseTarget(poseInOdom):
            TARGET_ID_SEQUENCE.append(marker.id)
            TARGET_SEQUENCE.append(poseInOdom)
            IS_ON_NAV_ROAD = True
            IS_ON_EXPLORE = False
        
        
def isPoseTarget(candidate):
    """Helper function to check if the given pose is in the right direction w.r.t. the current 
    position of the robot (or the approached old target). If the given pose is the next target 
    we want to find, then return true

    Args:
        candidate (PoseStamped): 

    Returns:
        bool:
    """
    oldTargetPose = TARGET_SEQUENCE[-1]
    oldTargetID = TARGET_ID_SEQUENCE[-1]
    old_x = oldTargetPose.pose.position.x
    old_y = oldTargetPose.pose.position.y
    new_x = candidate.pose.position.x
    new_y = candidate.pose.position.y
    vector_x = new_x - old_x
    vector_y = new_y - old_y

    if vector_y <= 0:
        vector_angle = -acos(vector_x / sqrt(vector_x**2 + vector_y**2))
    else:
        vector_angle = acos(vector_x / sqrt(vector_x**2 + vector_y**2))

    print("[isPoseTarget] vector_angle =", vector_angle)    
    
    if ENCODED_DIRS[oldTargetID][0] <= vector_angle <= ENCODED_DIRS[oldTargetID][1]:
        return True
    else:
        return False 


def fromCameraToOdom(markerInCamera):
    """Helper function to generate a PoseStamped in the odom (or map) coordinate system

    Args:
        markerInCamera (AlvarMaker):

    return:
        poseInOdom (PoseStamped):
    """
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(
        TARGET_FRAME_ID, 
        markerInCamera.header.frame_id[1:],
        rospy.Time(0), 
        rospy.Duration(1.0))        
    pose_transformed = tf2_geometry_msgs.do_transform_pose(markerInCamera.pose, transform)
    pose_transformed.header.frame_id = TARGET_FRAME_ID
    quaternionList = [pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w]
    eulerAngle = euler_from_quaternion(quaternionList)
    # debug message
    print("[fromCameraToOdom]", "xyz:", 
        round(pose_transformed.pose.position.x, 2), 
        round(pose_transformed.pose.position.y, 2), 
        round(pose_transformed.pose.position.z, 2), 
        "xyz angle:", 
        round(eulerAngle[0], 2), round(eulerAngle[1], 2), round(eulerAngle[2], 2))
    poseInOdom = pose_transformed
    return poseInOdom # a PoseStamped 

def isApproachOK(x, y, z):
    """

    Args:
        x (_type_): _description_
        y (_type_): _description_
        z (_type_): _description_

    Returns:
        bool: _description_
    """
    if sqrt(x**2 + y**2 + z**2) <= MIN_STOP_DIS:
        return True
    else:
        return False

def sendTheGoal(goal):
    '''This function sends the goal to move_base'''
    return None




def generateMoveBaseGoal(markerMsg):
    return 

def timerCallBack(event):
    """This is the timer callback function which handles the navigation process (sending nav goals or
    cmd_vel commands to the robot, based on the given stack of targets)

    Args:
        event (_type_): _description_
    """
    global MOVE_BASE_STATUS
    moveBaseActionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    moveBaseActionClient.wait_for_server()
    
    if len(TARGET_SEQUENCE) != 0:
        poseTarget = TARGET_SEQUENCE[-1]
    else:
        poseTarget = PoseStamped()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = TARGET_FRAME_ID
    goal.target_pose.header.stamp = rospy.Time.now()
    # afraid of the copy ;)
    goal.target_pose.pose.position.x = poseTarget.pose.position.x
    goal.target_pose.pose.position.y = poseTarget.pose.position.y
    goal.target_pose.pose.position.z = poseTarget.pose.position.z
    goal.target_pose.pose.orientation.x = -poseTarget.pose.orientation.x
    goal.target_pose.pose.orientation.y = -poseTarget.pose.orientation.y
    goal.target_pose.pose.orientation.z = -poseTarget.pose.orientation.z
    goal.target_pose.pose.orientation.w = -poseTarget.pose.orientation.w

    if not IS_ON_NAV_ROAD:
        moveBaseActionClient.cancel_all_goals()
        print("[timerCallBack] cancel all goals")
        twistMsg = Twist()
        twistMsg.angular.z = 0.2
        rospy.Publisher("/cmd_vel", Twist, queue_size=10).publish(twistMsg)
        return
    if IS_ON_NAV_ROAD and MOVE_BASE_STATUS == 1:
        # there is a goal currently
        print("[timerCallBack] not executing")
        return

    print("[timerCallBack] sending target", poseTarget)    
    moveBaseActionClient.send_goal(goal)
    # wait = moveBaseActionClient.wait_for_result()
    # if not wait:
        # rospy.logerr("Action server not available")
        # rospy.signal_shutdown("Action server not available")
    MOVE_BASE_STATUS = moveBaseActionClient.get_state()
    print(moveBaseActionClient.get_goal_status_text())
    

def main():
    rospy.init_node('marker_tracking_mappliess')
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, getTargetMarker)
    rospy.Timer(rospy.Duration(0.5), timerCallBack)
    rospy.spin()
    
if __name__ == "__main__":
    main()


