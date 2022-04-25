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
import tf, tf2_ros

def fromCameraToOdo(markerInCamera):
    '''This function makes the transformation from the marker position in 
    /virtual_camera_link to /odom
    
    markerInCamera: type: TODO
    markerInOdom: type: TODO

    '''

    markerInOdom = None
    return markerInOdom

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
    
    while not IS_STOP:
        if IS_ANOTHER_MARKER:
            # newGoal = transform
            # send newGoal to approach
            pass


