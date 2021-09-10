#!/usr/bin/env python

import sys
import rospy
import math
import cv2
import time

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pheonix.srv import*
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode , CommandTOL
from mavros_msgs.msg import State

taskCompleted = False
lastYaw = 0 # 0 meaning anticlockwise, 1 meaning clockwise. yaw positive means anticlockwise
groundPic = None
mode = None
armed = None
takeOff = False

def d2r(deg):
    return deg*3.14/180

def state_update(data):
    global mode, armed
    mode = data.mode
    armed = data.armed




def subscriber():
    rospy.loginfo('In subscriber')
    downImage = rospy.Subscriber('/camera/color/image_raw', Image, downCam)
    depthImage = rospy.Subscriber('/depth_camera/depth/image_raw', Image, move)
    rospy.Subscriber('/mavros/state', State, state_update)

    rospy.spin()

def downCam(message):
    global groundPic
    bridge = CvBridge()
    groundPic = bridge.imgmsg_to_cv2(message, "bgr8")

def aruco_detected():
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(groundPic, arucoDict, parameters=arucoParams)

    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            if markerID == 0:
                rospy.loginfo('aruco detected!!! moving towards it!!')
                cornersOfMarker = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = cornersOfMarker
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                coordinates = (cX, cY)
                rospy.loginfo("center of marker: x " + str(cX) + " y " + str(cY))
                return True, coordinates
 
    return False, (-1,-1)              
    
def move(message):
    global taskCompleted
    rospy.loginfo('In move!!')
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(message, "32FC1")
    
    data = np.asarray(cv_image)
    rospy.loginfo(cv_image[240][290:350])

    for i in range(640):
        if math.isnan(data[240][i]):
            data[240][i] = 0
        else:
            data[240][i] = 1

    zerosLeft = 0
    zerosRight = 0
    isSafe = True
    for i in range(640):
        if (i > 280 and i < 360) and data[240][i] == 1:
            isSafe = False
            break

        if data[240][i] == 0 and i < 320:
            zerosLeft = zerosLeft + 1
        elif data[240][i] == 0 and i > 320:
            zerosRight = zerosRight + 1

    rospy.loginfo(isSafe)
    rospy.loginfo(data[240][250:370])
    rospy.loginfo(len(data))
    rospy.loginfo("zeros R: " + str(zerosRight))
    rospy.loginfo("zeros L: " + str(zerosLeft))

    isArucoDetected, coordinates = aruco_detected() 

    if taskCompleted == True:
        publisher(1)

        rospy.loginfo('Done With Everything!!!')

    elif isArucoDetected == True:
        rospy.loginfo('Moving towards aruco!!!!')
        x,y = coordinates
        moveHorizontal = 0
        moveVertical = 0
        if (x > 290 and x < 350) and (y > 210 and y < 270):
            rospy.loginfo('In Landing part!!')
            land()
            taskCompleted = True
        if not (x > 290 and x < 350) == True:
            if x <= 290:
                moveHorizontal = 0.5
            else:
                moveHorizontal = -0.5
        if not (y > 210 and y < 270) == True:
            if y <= 210:
                moveVertical = 0.5
            else:
                moveVertical = -0.5

        if taskCompleted == False:
            publisher(0)
            move_to_destination(moveVertical,moveHorizontal,0,0)

    elif isSafe:
        move_to_destination(-2,0,0,0)
        publisher(0)

    else:
        if zerosLeft > zerosRight:
            move_to_destination(0,0,0,d2r(5))
        else:
            move_to_destination(0,0,0,-1*d2r(5))    
        publisher(0)


    cv2.imshow("depth_cam", cv_image)
    cv2.waitKey(3)
    rospy.loginfo("move sleeping now...")

def publisher(num):
    pub = rospy.Publisher('Aruco/message', String, queue_size = 10)
    msg_to_publish = String()
    if num == 1:
        string_to_pub = "Marker ID : 0, Landed"
    else:
        string_to_pub = "Marker ID: none, looking for marker"
    msg_to_publish.data = string_to_pub
    pub.publish(msg_to_publish)

def move_to_destination(x,y,z,yaw,frame_id='fcu_horiz'):
    rospy.loginfo('moving to destination...')
    rospy.wait_for_service('set_position')
    try:
        set_position = rospy.ServiceProxy('set_position', SetPosition)
        resp1 = set_position(x = x, y = y ,z = z, yaw = yaw, frame_id=frame_id)
        rospy.loginfo(resp1.success)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def land():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        resp1 = set_mode(custom_mode='LAND')
        # rospy.loginfo(resp1.success)
        # return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def takeoff():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        initiateTakeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        resp1 = set_mode(custom_mode='LAND')
        # rospy.loginfo(resp1.success)
        # return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def offboard_and_arm():
    set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    initiateTakeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

    global takeOff

    if mode != 'OFFBOARD':
        time.sleep(.3)
        rospy.loginfo('changing to OFFBOARD')
        ret = set_mode(base_mode=0, custom_mode='OFFBOARD')
        if not ret.mode_sent:
            return False
        start = time.time()
        while True:
            if mode == 'OFFBOARD':
                break
            if time.time() - start > 3:
                return False
    if not armed:
        rospy.loginfo('Arming')
        ret = arming(True)
        if not ret.success:
            return False
        start = time.time()
        while True:
            if armed:
                rospy.loginfo(armed)
                break
            if time.time() - start > 5:
                return False

    if takeOff == False:
        rospy.wait_for_service("/mavros/cmd/takeoff",timeout=30)
        ret = initiateTakeoff(altitude = 3)
        rospy.sleep(5)
        if not ret.success:
            return False
        start = time.time()
        while True:
            if ret:
                rospy.loginfo("Taking off")
                takeOff = True
                break
            if time.time() - start > 10:
                return False

    return True


if __name__ == "__main__":
    
    rospy.Subscriber('/mavros/state', State, state_update)
    # arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    # ret = arming(True)
    # if not ret.success:
    #     print("no suc")
    # start = time.time()
    # while True:
    #     if armed:
    #         rospy.loginfo(armed)
    #         break
    #     if time.time() - start > 5:
    #         print(False)
    #         break 
    k=0
    while k<100:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1


    rospy.loginfo("in moveDrone main!!")
    rospy.init_node('my_node_name')
    #offboard_and_arm()
    move_to_destination(0,0,15,0)
    # rospy.sleep(5)
    # move_to_destination(0,0,0,d2r(180))
    # rospy.sleep(5)
    # move_to_destination(-1,0,0,0)
    # rospy.sleep(5)
    #subscriber()