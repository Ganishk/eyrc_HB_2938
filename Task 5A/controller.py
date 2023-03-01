#!/usr/bin/env python3

import cv2 as cv
from cv2 import aruco
import numpy as np
import sys,math

import socket
import signal
from time import sleep

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

x_goals = [350,150,150,350]
y_goals = [300,300,150,150]
theta_goals = [0.25*PI,0.75*PI,-0.75*PI,-0.25*PI]

################################################################################ 
################################## FUNCTIONS ################################### 
################################################################################

def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
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

    while True:
        getPose()
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

establish()

################################################################################ 
#################################### IMAGE #####################################
################################################################################

arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)
arucoParams = aruco.DetectorParameters_create()

def getPose():
    global x,y,theta
    frame=cv.imread("pose2.png")
    frame=cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    (corners,marker_ids,reject) = aruco.detectMarkers(
        frame,arucoDict,parameters=arucoParams)

    if corners:
        if len(corners)<4: print("[!] Some Arucos are not in detected")
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
        result = cv.warpPerspective(frame,matrix,(500,500),flags=cv.INTER_LINEAR)

        (c,m,r) = aruco.detectMarkers( result, arucoDict,parameters=arucoParams)
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
                    print("Found the bot at", x,y,theta)

        cv.imshow('frame',result)
        key=cv.waitKey(1)

x_goal=x_goals[0]
y_goal=y_goals[0]
theta_goal=theta_goals[0]

x_goal = 355.5
y_goal = 198.5
theta_goal = PI/2

while True:
    p()

s.close()
cv.destroyAllWindows()

################################################################################ 
##################################### MAIN #####################################
################################################################################

def main():
    signal.signal(signal.SIGINT, signal_handler)
    establish()
    rospy.init_node('aruco_feedback_node')
    rospy.Subscriber('usb_cam/image_rect',Image,callback)
    rospy.spin()
