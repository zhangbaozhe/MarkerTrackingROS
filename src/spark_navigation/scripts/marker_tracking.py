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
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
import tf, tf2_ros, tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from math import pi, acos, sqrt, sin, cos
from time import sleep
from random import uniform


##########################################################################################
# Global constants
##########################################################################################
## id(int) : (lower bound, upper bound)(tuple)
ENCODED_DIRS = {0: (0.0, pi/4), 1: (pi/4, pi/2), 2: (pi/2, 3*pi/4), 3: (3*pi/4, pi), 
                4: (-pi, -3*pi/4), 5: (-3*pi/4, -pi/2), 6: (-pi/2, -pi/4), 7: (-pi/4, 0.0), 
                8: None}
## initial id
INIT_ID = 4
## minimum stop distance (m)
INFLATION_DIS = 0.2
MIN_STOP_DIS = 2 + INFLATION_DIS
MIN_DELIVER_DIS = 2.6
## target frame id
# TARGET_FRAME_ID = "map"
TARGET_FRAME_ID = "odom"
MIN_MAP_X = -9
MAX_MAP_X = 9
MIN_MAP_Y = -5
MAX_MAP_Y = 2.9
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
IS_ON_DELIVER = True
IS_ON_EXIT = False
## IMPORTANT!!! REMEBER TO UPDATE THE THREE BELOW AT THE SAME TIME
## This is the global target sequence containing the PoseStamped (in the odom or map coordinate) with the last of it as the new target
TARGET_SEQUENCE = []
## This is the global target ID sequence
TARGET_ID_SEQUENCE = []

NEW_ROBOT_POSE = PoseStamped()
ON_DELIVER_POSE = None

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
        TARGET_SEQUENCE.append(
            getTargetOutInflation(
                checkMarkerPoseBoundAndChange(fromCameraToOdom(marker))
            )
        )
        IS_ON_NAV_ROAD = True

def getNewRobotPose(data):
    """Callback function to update the robot position

    Args:
        data (PoseWithCovarianceStamped): _description_

    Returns:
        _type_: _description_
    """
    global NEW_ROBOT_POSE, IS_ON_EXIT, IS_ON_DELIVER, IS_ON_NAV_ROAD, IS_ON_EXPLORE
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(
        TARGET_FRAME_ID, 
        "map",
        rospy.Time(0), 
        rospy.Duration(1.0))  
    pose_transformed = tf2_geometry_msgs.do_transform_pose(data.pose, transform)
    pose_transformed.header.frame_id = TARGET_FRAME_ID

    if IS_ON_EXIT and abs(NEW_ROBOT_POSE.pose.position.x - pose_transformed.pose.position.x) < 0.00002:
        # stuck
        IS_ON_NAV_ROAD = False
        IS_ON_EXPLORE = True
        IS_ON_EXIT = False
        IS_ON_DELIVER = False
        msg = Twist()
        # rotate to the next target direction
        msg.angular.z = -(ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][0] + ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][1]) / 2 
        rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(msg)
        sleep(1)
        rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(Twist())


    NEW_ROBOT_POSE.header.frame_id = TARGET_FRAME_ID
    NEW_ROBOT_POSE.header.stamp = rospy.Time.now()
    NEW_ROBOT_POSE.pose.position.x = pose_transformed.pose.position.x
    NEW_ROBOT_POSE.pose.position.y = pose_transformed.pose.position.y
    NEW_ROBOT_POSE.pose.position.z = pose_transformed.pose.position.z
    NEW_ROBOT_POSE.pose.orientation.x = pose_transformed.pose.orientation.x
    NEW_ROBOT_POSE.pose.orientation.y = pose_transformed.pose.orientation.y
    NEW_ROBOT_POSE.pose.orientation.z = pose_transformed.pose.orientation.z
    NEW_ROBOT_POSE.pose.orientation.w = pose_transformed.pose.orientation.w

    
    
    print("[getNewRobotPose] ", NEW_ROBOT_POSE)

    # return NEW_ROBOT_POSE

def getTargetOutInflation(poseTarget):
    my_x = NEW_ROBOT_POSE.pose.position.x
    my_y = NEW_ROBOT_POSE.pose.position.y
    goal_x = poseTarget.pose.position.x
    goal_y = poseTarget.pose.position.y
    dis = sqrt((goal_y - my_y)**2 + (goal_x - my_x)**2)
    if dis < INFLATION_DIS:
        return poseTarget
    else:
        cos_ = (goal_x - my_x) / dis
        sin_ = (goal_y - my_y) / dis
        new_x = (dis - INFLATION_DIS) * cos_
        new_y = (dis - INFLATION_DIS) * sin_
        poseTarget.pose.position.x = my_x + new_x
        poseTarget.pose.position.y = my_y + new_y
        return poseTarget


def checkMarkerPoseBoundAndChange(markerPose):
    if markerPose.pose.position.x < MIN_MAP_X:
        markerPose.pose.position.x = MIN_MAP_X
    if markerPose.pose.position.x > MAX_MAP_X:
        markerPose.pose.position.x = MAX_MAP_X
    if markerPose.pose.position.y < MIN_MAP_Y:
        markerPose.pose.position.y = MIN_MAP_Y
    if markerPose.pose.position.y > MAX_MAP_Y:
        markerPose.pose.position.y = MAX_MAP_Y
    return markerPose
    


def getTargetMarker(markersInCamera):
    """ MAIN CALLBACK FUNCTION
    This function receives the detected markers in the camera frame and add
    new target in the target sequence

    Args:
        markersInCamera (AlvarMarkers): The markers in the virtual_camera_link frame
    """
    global NEW_ROBOT_POSE, TARGET_SEQUENCE, TARGET_ID_SEQUENCE 
    global IS_ON_NAV_ROAD, IS_ON_EXPLORE, IS_ON_DELIVER, IS_ON_EXIT

    print("[getTargetMarker] TARGET_ID_SEQUENCE", TARGET_ID_SEQUENCE)

    # getNewRobotPose()
    #print("[getTargetMarker]", "IS_ON_NAV_ROAD =", IS_ON_NAV_ROAD, 
    #        "IS_ON_EXPLORE =", IS_ON_EXPLORE, 
    #        "IS_ON_EXIT =", IS_ON_EXIT, 
    #        "IS_ON_DELIVER =", IS_ON_DELIVER)


    # init process
    if len(TARGET_ID_SEQUENCE) == 0:
        # may be buggy, but let's try first
        # print("[getTargetMarker] len(markersInCamera.markers) =", len(markersInCamera.markers))
        if len(markersInCamera.markers) == 1:
            initTarget(markersInCamera.markers[0])
    delta_x = NEW_ROBOT_POSE.pose.position.x - TARGET_SEQUENCE[-1].pose.position.x
    delta_y = NEW_ROBOT_POSE.pose.position.y - TARGET_SEQUENCE[-1].pose.position.y
    delta_z = NEW_ROBOT_POSE.pose.position.z - TARGET_SEQUENCE[-1].pose.position.z
    # this means the delivery is OK, and we should exit for the next move
    # DELIVERING -> EXIT
    print("[getTargetMarker] distance =", sqrt(delta_x**2 + delta_y**2 + delta_z**2))
    
    if isApproachOK(delta_x, delta_y, delta_z) and IS_ON_DELIVER:
        # print("[getTargetMarker] IS_ON_NAV_ROAD =", IS_ON_NAV_ROAD)
        IS_ON_NAV_ROAD = False
        IS_ON_EXPLORE = False
        IS_ON_EXIT = True
        IS_ON_DELIVER = False
    # EXIT -> EXPLORE
    if IS_ON_EXIT and sqrt(delta_x**2 + delta_y**2 + delta_z**2) > MIN_DELIVER_DIS:
        IS_ON_NAV_ROAD = False
        IS_ON_EXPLORE = True
        IS_ON_EXIT = False
        IS_ON_DELIVER = False
    # NAV_ROAD -> DELIVERING
    if IS_ON_NAV_ROAD and sqrt(delta_x**2 + delta_y**2 + delta_z**2) < MIN_DELIVER_DIS:
        IS_ON_NAV_ROAD = False
        IS_ON_EXPLORE = False
        IS_ON_EXIT = False
        IS_ON_DELIVER = True


    # main loop for adding the new target
    for marker in markersInCamera.markers:
        # print(marker.id)
        # this is the target
        if marker.id == TARGET_ID_SEQUENCE[-1]:
            # update if the new observed target is still in a good position
            tempPose = getTargetOutInflation(checkMarkerPoseBoundAndChange(fromCameraToOdom(marker)))
            if len(TARGET_ID_SEQUENCE) > 1 and isPoseTarget(tempPose, TARGET_ID_SEQUENCE[-2], TARGET_SEQUENCE[-2]):
                # TARGET_SEQUENCE[-1] = tempPose
                pass
            delta_x = NEW_ROBOT_POSE.pose.position.x - TARGET_SEQUENCE[-1].pose.position.x
            delta_y = NEW_ROBOT_POSE.pose.position.y - TARGET_SEQUENCE[-1].pose.position.y
            delta_z = NEW_ROBOT_POSE.pose.position.z - TARGET_SEQUENCE[-1].pose.position.z
            # this means the delivery is OK, and we should exit for the next move
            # DELIVERING -> EXIT
            print("[getTargetMarker] distance =", sqrt(delta_x**2 + delta_y**2 + delta_z**2))
            
            if isApproachOK(delta_x, delta_y, delta_z) and IS_ON_DELIVER:
                # print("[getTargetMarker] IS_ON_NAV_ROAD =", IS_ON_NAV_ROAD)
                IS_ON_NAV_ROAD = False
                IS_ON_EXPLORE = False
                IS_ON_EXIT = True
                IS_ON_DELIVER = False
                break
            # EXIT -> EXPLORE
            if IS_ON_EXIT and sqrt(delta_x**2 + delta_y**2 + delta_z**2) > MIN_DELIVER_DIS:
                IS_ON_NAV_ROAD = False
                IS_ON_EXPLORE = True
                IS_ON_EXIT = False
                IS_ON_DELIVER = False
                break
            # NAV_ROAD -> DELIVERING
            if IS_ON_NAV_ROAD and sqrt(delta_x**2 + delta_y**2 + delta_z**2) < MIN_DELIVER_DIS:
                IS_ON_NAV_ROAD = False
                IS_ON_EXPLORE = False
                IS_ON_EXIT = False
                IS_ON_DELIVER = True
                break
        

        if marker.id in TARGET_ID_SEQUENCE:
            # we find the old one
            continue

        # this pose is ready to be added, a candidate target
        poseInOdom = getTargetOutInflation(checkMarkerPoseBoundAndChange(fromCameraToOdom(marker)))
        # if the checking process of the candidate pose is OK 
        # then we put the PoseStamped in the SEQUENCE
        # EXPLORE -> NAV_ROAD
        if (IS_ON_EXPLORE or IS_ON_EXIT) and isPoseTarget(poseInOdom, TARGET_ID_SEQUENCE[-1], TARGET_SEQUENCE[-1]):
            TARGET_ID_SEQUENCE.append(marker.id)
            TARGET_SEQUENCE.append(poseInOdom)
            IS_ON_NAV_ROAD = True
            IS_ON_EXPLORE = False
            IS_ON_EXIT = False
            IS_ON_DELIVER = False
        
        
def isPoseTarget(candidate, oldTargetID, oldTargetPose):
    """Helper function to check if the given pose is in the right direction w.r.t. the current 
    position of the robot (or the approached old target). If the given pose is the next target 
    we want to find, then return true

    Args:
        candidate (PoseStamped): 

    Returns:
        bool:
    """
    old_x = oldTargetPose.pose.position.x
    old_y = oldTargetPose.pose.position.y
    new_x = candidate.pose.position.x
    new_y = candidate.pose.position.y
    vector_x = new_x - old_x
    vector_y = new_y - old_y
    # print("[isPoseTarget] oldTargetID =", oldTargetID)
    print("[isPoseTarget]", "vector_x =", vector_x, "vector_y =", vector_y)

    if vector_y <= 0:
        vector_angle = -acos(vector_x / sqrt(vector_x**2 + vector_y**2))
    else:
        vector_angle = acos(vector_x / sqrt(vector_x**2 + vector_y**2))

    # print("[isPoseTarget] vector_angle =", vector_angle)    
    
    if ENCODED_DIRS[oldTargetID][0] <= vector_angle <= ENCODED_DIRS[oldTargetID][1] \
        and ((NEW_ROBOT_POSE.pose.position.x-new_x)**2 + (NEW_ROBOT_POSE.pose.position.y-new_y)**2) < 40:
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
    # print("[fromCameraToOdom]", "xyz:", 
    #    round(pose_transformed.pose.position.x, 2), 
    #     round(pose_transformed.pose.position.y, 2), 
    #    round(pose_transformed.pose.position.z, 2), 
    #    "xyz angle:", 
    #    round(eulerAngle[0], 2), round(eulerAngle[1], 2), round(eulerAngle[2], 2))
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

def generateMoveBaseGoal(poseTarget):
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
    return goal


def timerCallBack(event):
    """This is the timer callback function which handles the navigation process (sending nav goals or
    cmd_vel commands to the robot, based on the given stack of targets)

    Args:
        event (_type_): _description_
    """
    global ON_DELIVER_POSE, IS_ON_EXPLORE
    moveBaseActionClient = actionlib.ActionClient('move_base', MoveBaseAction)
    moveBaseActionClient.wait_for_server()
    # print(ON_DELIVER_POSE)
    if len(TARGET_SEQUENCE) != 0:
        poseTarget = TARGET_SEQUENCE[-1]
    else:
        poseTarget = PoseStamped()
    goal = generateMoveBaseGoal(poseTarget)
    # print("[timerCallBack] state =", moveBaseActionClient.get_state())
    print("[timerCallBack] goal =", goal)
    if IS_ON_DELIVER:
        print("++++++++++++++++++ DELIVER")
        # store the last pose
        if ON_DELIVER_POSE == None:
            ON_DELIVER_POSE = PoseStamped()
            ON_DELIVER_POSE.header.frame_id = TARGET_FRAME_ID
            ON_DELIVER_POSE.header.stamp = rospy.Time.now()
            ON_DELIVER_POSE.pose.position.x = NEW_ROBOT_POSE.pose.position.x
            ON_DELIVER_POSE.pose.position.y = NEW_ROBOT_POSE.pose.position.y
            ON_DELIVER_POSE.pose.position.z = NEW_ROBOT_POSE.pose.position.z
            ON_DELIVER_POSE.pose.orientation.x = NEW_ROBOT_POSE.pose.orientation.x
            ON_DELIVER_POSE.pose.orientation.y = NEW_ROBOT_POSE.pose.orientation.y
            ON_DELIVER_POSE.pose.orientation.z = NEW_ROBOT_POSE.pose.orientation.z
            ON_DELIVER_POSE.pose.orientation.w = NEW_ROBOT_POSE.pose.orientation.w
            
        moveBaseActionClient.send_goal(goal)
        sleep(3)
        moveBaseActionClient.stop()
        
        return

    if IS_ON_EXIT:
        print("++++++++++++++++++ EXIT")
        if ON_DELIVER_POSE != None:
            # moveBaseActionClient.cancel_all_goals()
            moveBaseActionClient.send_goal(generateMoveBaseGoal(ON_DELIVER_POSE))
            sleep(3)
            moveBaseActionClient.stop()
            ON_DELIVER_POSE = None
            '''
            msg = Twist()
            msg.linear.x = -0.5
            rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(msg)
            sleep(1)
            rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(Twist())
            '''
        else: 
            IS_ON_EXPLORE = True
            return
    
    if IS_ON_EXPLORE:
        print("++++++++++++++++++ EXPLORE")
        # moveBaseActionClient.stop()
        # msg = Twist()
        # rotate to the next target direction
        # msg.angular.z = (ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][0] + ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][1]) / 2 
        # rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(msg)
        # sleep(1)
        # rospy.Publisher("/cmd_vel", Twist, queue_size=1).publish(Twist())
        norm = 4
        #rand_angle = uniform(
        #    (ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][0] + ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][1]) / 2, 
        #    ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][1])
        rand_angle = ENCODED_DIRS[TARGET_ID_SEQUENCE[-1]][1]
        exploreGoal = MoveBaseGoal()
        exploreGoal.target_pose.header.frame_id = TARGET_FRAME_ID
        exploreGoal.target_pose.header.stamp = rospy.Time.now()
        exploreGoal.target_pose.pose.position.x = norm * cos(rand_angle) + NEW_ROBOT_POSE.pose.position.x
        exploreGoal.target_pose.pose.position.y = norm * sin(rand_angle) + NEW_ROBOT_POSE.pose.position.y
        exploreGoal.target_pose.pose.position.z = 0
        q = tf.transformations.quaternion_from_euler(0, 0, rand_angle)
        exploreGoal.target_pose.pose.orientation.x = q[0]
        exploreGoal.target_pose.pose.orientation.y = q[1]
        exploreGoal.target_pose.pose.orientation.z = q[2]
        exploreGoal.target_pose.pose.orientation.w = q[3]
        try:
            print("[timerCallBack] exploreGoal =", exploreGoal)
            moveBaseActionClient.send_goal(exploreGoal)
            sleep(3)
            moveBaseActionClient.stop()
        except Exception as e:
            print(e)
            print("[timerCallBack] exploreGoal =", exploreGoal)
        return
    
    if IS_ON_NAV_ROAD:
        print("++++++++++++++++++ NAV")
        moveBaseActionClient.send_goal(goal)
        sleep(5)
        moveBaseActionClient.stop()
        return 


def main():
    rospy.init_node('marker_tracking')
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, getTargetMarker)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, getNewRobotPose)
    rospy.Timer(rospy.Duration(2), timerCallBack)
    rospy.spin()
    
if __name__ == "__main__":
    main()


