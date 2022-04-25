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

# Global variables
IS_STOP = False

def fromCameraToOdom(markerInCamera):
    '''This function makes the transformation from the marker position in 
    /virtual_camera_link to /odom
    
    markerInCamera: type: AlvarMarkers
    markerInOdom: type: TODO

    '''
    # testing code 
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    if len(markerInCamera.markers) != 0:
        for marker in markerInCamera.markers:
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            transform = tf_buffer.lookup_transform(
                "odom", 
                marker.header.frame_id[1:],
                rospy.Time(0), 
                rospy.Duration(1.0))        
            pose_transformed = tf2_geometry_msgs.do_transform_pose(marker.pose, transform)
            print(pose_transformed.pose)
            markerInOdom = pose_transformed
    # return markerInOdom

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
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, fromCameraToOdom)
    rospy.spin()
    
if __name__ == "__main__":
    main()


