#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:  HB#2938	[ Team-ID ]
# Author List: Ganishk D, Srivattsan S, Susmitha S, Janam Khandhelwal	[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node

################################################################################ 
################################### MODULES ####################################
################################################################################
import cv2 as cv
from cv2 import aruco
import numpy as np

import sys,math
import socket
import signal
from time import sleep

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_basics.msg import aruco_data
from std_msgs.msg import String,Int32
################################################################################ 
################################## VARIABLES ################################### 
################################################################################

global conn,s

IP      = "192.168.1.123"
PORT    = 8002
PI      = 3.141592653589793
dTOL    = 3
aTOL    = 0.05
Kpd     = 600
KpA     = 8000
msg     = ""
flag    = True
N       = 150

x_goals = []
y_goals = []
theta_goals = []
x,y,theta = (0,0,0)
count = 0

################################################################################ 
################################## FUNCTIONS ################################### 
################################################################################

def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    cv.destroyAllWindows()
    print("cleanup done")

def switchPen():
    global penData,x_goal,y_goal,x_goals,y_goals,theta_goal,theta_goals,msg
    if penData.data:
        # Up the pen
        msg="0 0 0 2"
        penData.data = 0
        print("Take the pen")
    else:
        # Down the pen
        x_goal = x_goals.pop(0)
        y_goal = y_goals.pop(0)
        theta_goal = theta_goals.pop(0)
        p()
        penData.data = 1
        msg="0 0 0 1"
        teleport()
        print("lower the pen")

    penPub.publish(penData)
################################################################################ 
############################## INVERSE KINEMATICS ##############################
################################################################################

def inverse_kinematics(Vx,Vy,W):
    global msg
    v1 = Vx - W
    v2 = -0.5*Vx - 0.866*Vy - W
    v3 = -0.5*Vx + 0.866*Vy - W
    # Using less decimal places to send less data, which increases transmission
    # speed
    msg = f"{v1:.3f} {v2:.3f} {v3:.3f}\n"
    teleport()

################################################################################ 
################################## CONTROLLER ##################################
################################################################################

def p():
    #print(x_goal,y_goal,theta_goal)
    
    exprev = 0
    eyprev = 0
    etprev = 0
    Tp = 1000
    Tpa = 20000
    while True:
        e_x = x_goal - x
        e_y = y_goal - y
        e_theta = theta_goal - theta
        E_x = e_x*math.cos(theta) + e_y*math.sin(theta)
        E_y = e_y*math.cos(theta) - e_x*math.sin(theta)

        if abs(E_x) <= dTOL: Kpx = 0
        else: Kpx = Kpd

        if abs(E_y) <= dTOL: Kpy = 0
        else: Kpy = Kpd

        if abs(e_theta) <= aTOL: Kpa = 0
        else: Kpa = KpA
        
        Vx = Kpx*(E_x + Tp*exprev)
        Vy = Kpy*(E_y + Tp*eyprev)
        W = Kpa*(e_theta + Tpa*etprev)
        exprev = E_x
        eyprev = E_y

        inverse_kinematics(Vx,Vy,W)
        #print(Vx,Vy,W,end="-->")

        if not(abs(E_x) > dTOL or abs(E_y) > dTOL or abs(e_theta) > aTOL):
            break

################################################################################ 
#################################### SOCKET ####################################
################################################################################

def teleport():
    #print(msg)
    conn.sendall(msg.encode())
    conn.recv(1)

def establish():
    global conn,s
    print("Starting from",IP)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((IP,PORT))
    rospy.loginfo("Binded successfully.")
    s.listen()
    conn,addr = s.accept()

    rospy.loginfo(f"Connected by {addr} at {IP}:{PORT}")

################################################################################ 
#################################### IMAGE #####################################
################################################################################

arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)
arucoParams = aruco.DetectorParameters_create()

def perspection(frame):
    global matrix,flag

    rospy.loginfo("First trying to locate the aruco")
    (corners,marker_ids,reject) = aruco.detectMarkers(
        frame,arucoDict,parameters=arucoParams)

    while True:
        if corners:
            if len(corners)<4:
                rospy.loginfo("[!] Some Arucos are not in detected")
                break
            pts = [0,0,0,0]
            for no,corner in zip(marker_ids,corners):
                (tL,tR,bR,bL) = corner[0]
                if no==4:
                    pts[0] = bL
                elif no==8:
                    pts[1] = tL
                elif no==10:
                    pts[2] = tR
                elif no==12:
                    pts[3] = bR
            pts = np.float32(pts)
            brds =  np.float32([[0,500],[0,0],[500,0],[500,500]])
            matrix = cv.getPerspectiveTransform(pts,brds)
            flag = False
            break

def getPose(data):
    global x,y,theta,matrix,aruco_msg

    br = CvBridge()
    frame = br.imgmsg_to_cv2(data,"mono8")
    #frame = cv.imread("pose2.png")
    #frame=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)

    if flag:
        perspection(frame)
        return
    result = cv.warpPerspective(frame,matrix,(500,500),flags=cv.INTER_LINEAR)
    (c,m,r) = aruco.detectMarkers(result, arucoDict,parameters=arucoParams)
    if c:
        for no,corner in zip(m,c):
            if no==15:
                (tL,tR,bR,bL) = corner[0]
                x = (tL[0] + tR[0] + bR[0] + bL[0])/4 #-#
                y = 500 - (tL[1] + tR[1] + bR[1] + bL[1])/4 #-#
                rC =((bR[0]+tR[0])/2, 500 - (bR[1]+tR[1])/2 )
                dX = rC[0] - x
                dY = rC[1] - y
                theta = math.atan2(dY,dX) #-#
                #print(f"Found the bot at x={x},y={y},\u03b8={theta}")

                aruco_msg.x     = x
                aruco_msg.y     = y
                aruco_msg.theta = theta
                aruco_publisher.publish(aruco_msg)


        cv.putText(result,f"x: {x}",(30,40),cv.FONT_HERSHEY_SIMPLEX,0.5,0xff,1)
        cv.putText(result,f"y: {y}",(30,60),cv.FONT_HERSHEY_SIMPLEX,0.5,0xff,1)
        cv.putText(result,f"t: {theta:.2f}",(30,80),cv.FONT_HERSHEY_SIMPLEX,0.5,0xff,1)
    cv.imshow('arena',result)
    key=cv.waitKey(1)

################################################################################
################################ FUNCTION MODE #################################
################################################################################

def function():
    global KpA, Kpd, dTol, aTOL

    dTOL    = 2
    aTOL    = 0.1
    Kpd     = 280
    KpA     = 8700

    x_t = lambda t: 200*(math.cos(t))+250
    y_t = lambda t: 100*(math.sin(2*t))+250
    theta_t = lambda t: PI*0.25*math.sin(t)
    tmin = 0
    tmax = 2*PI
    dt = (tmax-tmin)/N
    x_goals.clear()
    y_goals.clear()
    theta_goals.clear()
    rospy.Subscriber('usb_cam/image_rect',Image,getPose)
    for i in range(N+1):
        x_goals.append(x_t(i*dt))
        y_goals.append(y_t(i*dt))
        theta_goals.append(theta_t(i*dt))
    interpolate()
    
################################################################################
################################## IMAGE MODE ##################################
################################################################################

def image(img_name,RESOLUTION=1):
    global count,cData,penData,taskStatus
    global KpA, Kpd, dTol, aTOL
    global x_goals,y_goals,theta_goals

    dTOL    = 0.25
    aTOL    = 0.08
    Kpd     = 15000
    KpA     = 30000

    img = cv.imread(img_name)
    img = cv.resize(img,(500,500),interpolation=cv.INTER_AREA)
    gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)

    blurred = cv.GaussianBlur(gray,(3,3),0)
    edged = cv.Canny(blurred,10,100)

    contours,hierarchy = cv.findContours(edged,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)


    xListFinal,yListFinal = [],[]
    for x in range(2,len(contours),2):
        x_goals.clear()
        y_goals.clear()
        theta_goals.clear()
        for i in range(len(contours[x])):
            x_goals.append(contours[x][i][0][0])
            y_goals.append(500-contours[x][i][0][1])
            theta_goals.append(0)
            count+=1
        rospy.loginfo(len(x_goals))
        #Assume the goals are not empty from the figure

        xListFinal.append(list(x_goals))
        yListFinal.append(list(y_goals))


        x_goals.append(x_goals[0])
        y_goals.append(y_goals[0])
        theta_goals.append(0)
        """
        x_goals=x_goals[::RESOLUTION]
        y_goals=y_goals[::RESOLUTION]

        switchPen()

        interpolate()

        switchPen()
        #"""

    cData.data = str([xListFinal,yListFinal])
    while contourPub.get_num_connections() <= 0: pass
    contourPub.publish(cData)

    taskStatus.data = 0
    while taskStatusPub.get_num_connections()<=0: pass
    while penPub.get_num_connections()<=0: pass
    while aruco_publisher.get_num_connections()<=0: pass
    taskStatusPub.publish(taskStatus)

    rospy.Subscriber('usb_cam/image_rect',Image,getPose)
    for x_goals,y_goals in zip(xListFinal,yListFinal):
        x_goals=x_goals[::RESOLUTION]
        y_goals=y_goals[::RESOLUTION]
        x_goals.append(x_goals[0])
        y_goals.append(y_goals[0])
        theta_goals = [0]*len(x_goals)
        switchPen()
        interpolate()
        switchPen()

    taskStatus.data = 1
    taskStatusPub.publish(taskStatus)
    
################################################################################ 
##################################### MAIN #####################################
################################################################################

def interpolate():
    global x_goal,y_goal,theta_goal
    while (not rospy.is_shutdown()) and x_goals:
        x_goal = x_goals.pop(0)
        y_goal = y_goals.pop(0)
        theta_goal = theta_goals.pop(0)
        p()

def main():
    global x_goal,y_goal,theta_goal,x,y,theta
    global aruco_publisher,contourPub,penPub,taskStatusPub
    global aruco_msg,cData,penData,taskStatus

    establish()
    rospy.init_node('aruco_node')

    # Publishers
    aruco_publisher = rospy.Publisher('/detected_aruco',aruco_data,queue_size=10)
    aruco_msg = aruco_data()

    contourPub = rospy.Publisher('/contours',String,queue_size=10)
    cData = String()

    penPub = rospy.Publisher('/penStatus',Int32,queue_size=10)
    penData = Int32()

    taskStatusPub = rospy.Publisher('/taskStatus',Int32,queue_size=10)
    taskStatus = Int32()

   # taskStatus.data = 0
   # taskStatusPub.publish(taskStatus)

    # Subscribers
    #rospy.Subscriber('endSignal',Int32,cleanup)

    # wait until the 1st feedback

    while not rospy.is_shutdown():
        changeMode(2)
        break

def changeMode(mode):
    if mode==1:
        image("/home/ganishk/catkin_ws/src/hola_bot/scripts/snapchat.png",8)
    elif mode==2:
        function()
    elif mode==3:
        image("/home/ganishk/catkin_ws/src/hola_bot/scripts/smile.png",2)

if __name__=="__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
