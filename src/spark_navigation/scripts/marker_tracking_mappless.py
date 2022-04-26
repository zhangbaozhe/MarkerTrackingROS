#!/usr/bin/env python

'''
Author: Baozhe Zhang
Date: April 25, 2022
Usage: ECE4310 Project 1
'''

import rospy 
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker # Marker tag
from geometry_msgs.msg import PoseStamped
import tf, tf2_ros, tf2_geometry_msgs
from math import pi

# Global constants
## id(int) : (lower bound, upper bound)(tuple)
ENCODED_DIRS = {0: (0, pi/4), 1: (pi/4, pi/2), 2: (pi/2, 3*pi/4), 3: (3*pi/4, pi), 
                4: (-pi, -3*pi/4), 5: (-3*pi/4, -pi/2), 6: (-pi/2, -pi/4), 7: (-pi/4, 0), 
                8: None}

# Global variables
IS_STOP = False
## This is the global marker pose which lies in odom or (TODO map) 
## It is the most updated target in each iteration of the navigation process
POSE_TARGET = AlvarMarker().pose 
## This is the global target sequence with the last of it as the new target
TARGET_SEQUENCE = []

def getTargetMarker(markersInCamera):
    """This function receives the detected markers in the camera frame and add
    new target in the target sequence

    Args:
        markersInCamera (AlvarMarkers): The markers in the virtual_camera_link frame
    """
    global TARGET_SEQUENCE
    for marker in markersInCamera.markers:
        pass


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
        "odom", 
        markerInCamera.header.frame_id[1:],
        rospy.Time(0), 
        rospy.Duration(1.0))        
    pose_transformed = tf2_geometry_msgs.do_transform_pose(markerInCamera.pose, transform)
    print("[fromCameraToOdom]", pose_transformed.pose)
    poseInOdom = pose_transformed
    return poseInOdom # a PoseStamped 

def isApproachOK(markerInCamera):
    '''This function returns true if the robot is colse enough with the marker'''
    threshold = 4 # meter
    # norm of the pose
    # TODO
    return True

def sendTheGoal(goal):
    '''This function sends the goal to move_base'''
    return None


def checkNewMarker(markers):
    '''Checks if a new marker is detected'''
    newMarkerPose = None
    return newMarkerPose


def generateMoveBaseGoal(markerMsg):
    return 

def main():
    rospy.init_node('marker_tracking_mappliess')
    # rospy.Subscriber('/ar_pose_marker', AlvarMarkers, fromCameraToOdom)
    rospy.spin()
    
if __name__ == "__main__":
    main()


