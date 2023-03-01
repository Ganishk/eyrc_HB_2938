#!/usr/bin/env python3

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
################################################################################ 
################################## VARIABLES ################################### 
################################################################################

global conn,s

IP      = "127.0.0.1"
PORT    = 8002
PI      = 3.141592653589793
dTOL    = 5
aTOL    = 1
Kpd     = 20
msg     = ""
flag    = True

x_goals = [350,150,150,350]
y_goals = [300,300,150,150]
theta_goals = [0.25*PI,0.75*PI,-0.75*PI,-0.25*PI]
#x,y,theta = (0,0,0)

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

def goals():
    pass

def callback():
    pass

################################################################################ 
############################## INVERSE KINEMATICS ##############################
################################################################################

def inverse_kinematics(Vx,Vy,W):
    global msg
    v1 = Vx - W
    v2 = -0.5*Vx - 0.866*Vy - W
    v3 = -0.5*Vx + 0.866*Vy - W
    msg = f"{v1} {v2} {v3}"
    teleport()

################################################################################ 
################################## CONTROLLER ##################################
################################################################################

def p():
    global theta,x_goal,y_goal,theta_goal,x,y,theta
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
        else: Kpa = 30
        
        Vx = Kpx*E_x
        Vy = Kpy*E_y
        W = Kpa*e_theta

        inverse_kinematics(Vx,Vy,W)

        if not(abs(E_x) > dTOL or abs(E_y) > dTOL or abs(e_theta) > aTOL):
            sleep(1)
            break

################################################################################ 
#################################### SOCKET ####################################
################################################################################

def teleport():
    global conn,msg
    print(msg)
    conn.sendall(msg.encode())

def establish():
    global conn,s
    print("Starting from",IP)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((IP,PORT))
    print("Binded successfully.")
    s.listen()
    conn, addr = s.accept()

    print(f"Connected by {addr} at {IP}:{PORT}")

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
                print("[!] Some Arucos are not in detected")
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
    global x,y,theta,matrix

    #rospy.loginfo("Locating bot")
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
                #tC =((tL[0]+tR[0])/2, 500 - (tL[1]+tR[1])/2 )
                rC =((bR[0]+tR[0])/2, 500 - (bR[1]+tR[1])/2 )
                dX = rC[0] - x
                dY = rC[1] - y
                theta = math.atan2(dY,dX) #-#
                #y = 500 - y #-#
                print(f"Found the bot at x={x},y={y},\u03b8={theta}", x,y,theta)

    cv.putText(result,f"x: {x}",(30,40),cv.FONT_HERSHEY_SIMPLEX,0.5,(0,),2)
    cv.putText(result,f"y: {y}",(30,60),cv.FONT_HERSHEY_SIMPLEX,0.5,0xff,2)
    cv.putText(result,f"t: {theta:.2f}",(30,80),cv.FONT_HERSHEY_SIMPLEX,0.5,(0xff,),1)
    cv.imshow('arena',result)
    key=cv.waitKey(1)

################################################################################ 
##################################### MAIN #####################################
################################################################################

def main():
    global x_goal,y_goal,theta_goal,x,y,theta
    establish()
    rospy.init_node('aruco_feedback_node')
    rospy.Subscriber('usb_cam/image_rect',Image,getPose)

    # wait until the 1st feedback
    x = 0
    while not x: print(".",end="")
    else: print("Starting from",x,y,theta)

    while (not rospy.is_shutdown()) and x_goals:
        x_goal = x_goals.pop(0)
        y_goal = y_goals.pop(0)
        theta_goal = theta_goals.pop(0)
        p()

if __name__=="__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
