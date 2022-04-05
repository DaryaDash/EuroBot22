#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import rospy
from std_msgs.msg import Float64


marker_size = 10
# cameraMatrix = np.loadtxt('cameraMatrix.txt', delimiter=',')
# cameraDisortion = np.loadtxt('cameraDistortion.txt', delimiter=',')

def findArucomarkers(img, markerSize=4, totalMarkers=100, draw=True):        #находит маркеры на изображении
    imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    key = getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bboxs, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)
    # print(ids)

    if draw:
        aruco.drawDetectedMarkers(img, bboxs)
    return [bboxs, ids]

'''
def augmentAruco(bbox, id, img, imgAug, draw_id=True):          #добавляет картинку на экран
    tl = int(bbox[0][0][0]), int(bbox[0][0][1])
    tr = bbox[0][1][0], bbox[0][1][1]
    br = bbox[0][2][0], bbox[0][2][1]
    bl = bbox[0][3][0], bbox[0][3][1]

    h, w, c = imgAug.shape
    
    pts1 = np.array([tl, tr, br, bl])
    pts2 = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    matrix, _ = cv.findHomography(pts2, pts1)
    imgOut = cv.warpPerspective(imgAug, matrix, (img.shape[1], img.shape[0]))
    cv.fillConvexPoly(img, pts1.astype(int), (0,0,0))
    imgOut = img + imgOut
    
    if draw_id:
        cv.putText(imgOut, str(id), tl, cv.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

    return imgOut'''


def makeCoordinates(img, bbox, id, showImg=False):             #рисует картинки, выводит координаты
    x = (bbox[0][0][0] + bbox[0][1][0] + bbox[0][2][0] + bbox[0][3][0]) / 4
    y = (bbox[0][0][1] + bbox[0][1][1] + bbox[0][2][1] + bbox[0][3][1]) / 4
    center_on_img = [x, y]
    if id == 47:
        element = 'red'
    elif id == 13:
        element = 'blue'
    elif id == 36:
        element = 'green'
    elif id == 17:
        element = 'stone'
    else:
        element = 'not_detected'
    coordinate = [[element, center_on_img]]
    if showImg:
        if element == 'red':
            imgAug = cv.imread('images/red.png')
        elif element == 'blue':
            imgAug = cv.imread('images/blue.png')
        elif element == 'green':
            imgAug = cv.imread('images/green.png')
        elif element == 'stone':
            imgAug = cv.imread('images/stone.png')
        h, w, c = imgAug.shape
        tl = int(bbox[0][0][0]), int(bbox[0][0][1])
        tr = bbox[0][1][0], bbox[0][1][1]
        br = bbox[0][2][0], bbox[0][2][1]
        bl = bbox[0][3][0], bbox[0][3][1]
        pts1 = np.array([tl, tr, br, bl])
        pts2 = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        matrix, _ = cv.findHomography(pts2, pts1)
        imgOut = cv.warpPerspective(imgAug, matrix, (img.shape[1], img.shape[0]))
        cv.fillConvexPoly(img, pts1.astype(int), (0,0,0))
        imgOut = img + imgOut
        ret = aruco.estimatePoseSingleMarkers(bbox, marker_size, cameraMatrix, cameraDisortion)
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]           #tvec это положение объекта, a rvec это направление его
        # print('r ',rvec)
        # print('t ',tvec)

        aruco.drawAxis(imgOut, cameraMatrix, cameraDisortion, rvec, tvec, 10)
        
        return coordinate, imgOut
    return coordinate, img
    




def main():
    cap = cv.VideoCapture(0)
    # imgAug = cv.imread('0.jpg')
    rospy.init_node('opencv')
    pub_aruco_stat = rospy.Publisher('aruco_stat', Float64, queue_size=10)
    while True:
        success, img = cap.read()
        arucoFound = findArucomarkers(img)
        coordinaties = []
        if len(arucoFound[0]) != 0:
            for bbox, id in zip(arucoFound[0], arucoFound[1]):
                coordinate, img = makeCoordinates(img, bbox, id)
                coordinaties += coordinate
            pub_aruco_stat.publish(coordinaties[0][1][0]-320)
            print(coordinaties)
        cv.imshow('Image', img)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main()
