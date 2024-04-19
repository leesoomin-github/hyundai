#!/usr/bin/python3.8
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray

class Camera:
    lmax = 200
    rmin = 1080

    lavg = 0
    ravg = 0

    def __init__(self):
        rospy.init_node('camera', anonymous = True)

        self.image_sub = rospy.Subscriber("/Camera_1/image_raw/compressed", CompressedImage, self.callback)
        #self.image_publish = rospy.Publisher("/new_image", Image, queue_size = 1)
        self.pub = rospy.Publisher('Camera_Control',Float32MultiArray,queue_size=10) 

        rospy.spin()

    def main(self, img_bgr):
        bird_eye_img = self.bird_eye_view(img_bgr)
        result = self.taeyang(bird_eye_img)
        self.Pub_Control_msg(result)
 
        #cv2.imshow("Image window", img_bgr)
        cv2.imshow("bird_eye_img", bird_eye_img)
        cv2.waitKey(1)
    
    def callback(self, data):
        
        np_arr = np.frombuffer(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.main(img_bgr)

    def publish_imge(self, imgdata):

        image_temp = Image()
        header = Header(stamp = rospy.Time.now())
        header.frame_id = 'camera_img'
        image_temp.height = imgdata.shape[0]
        image_temp.width = imgdata.shape[1]
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()

        image_temp.header = header

        return image_temp

    def bird_eye_view(self, img_bgr):

        view_img = img_bgr.copy()
        p1 =  [0, 370]  # 좌상
        p2 =  [1280, 370] # 우상
        p3 =  [1280, 720] # 우하 
        p4 =  [0, 720]  # 좌하  

        corner_points_arr = np.float32([p1, p2, p3, p4])
        height, width = view_img.shape[:2]

        image_p1 = [0, 0]
        image_p2 = [width, 0]
        image_p3 = [width, height]
        image_p4 = [0, height]

        image_params = np.float32([image_p1, image_p2, image_p3, image_p4])

        mat = cv2.getPerspectiveTransform(corner_points_arr, image_params)
    
        image_transformed = cv2.warpPerspective(view_img, mat, (width, height))

        return image_transformed

    def taeyang(self, dst):
        left = []
        right = []
        lext = [0]
        rext = [1280]

        src1 = self.Canny(dst)
        lines = cv2.HoughLinesP(src1, 1, np.pi/180, 50, None, 20, 5)
        #cv2.imshow('canny', self.src1)
        
        if lines is None:
            lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            a = (y2-y1)/float(x2-x1)
            b = y1-(a*x1)
            cv2.line(dst, (x1,y1), (x2, y2), (0,0,255), 2) #차선 직선 표시

            cx = (135-b)//a #교점 좌표 구하기
            cy = 135
            
            if a < -0.15 and x2 < 640 and cx > 0 and cx < 640:
                print(a)
                left.append(cx)  #왼쪽 군집
                
            elif a > 0.15 and x1 > 640 and cx < 1280 and cx > 640:
                right.append(cx) #오른쪽 군집
                
        if left != []:
            if left[0] != 0:
                lext = left  #left 값을 lext에 저장


        
        if right != []:
            if right[0] != 0:
                rext = right  #right 값을 rext에 저장

        if abs(max(lext)-self.lmax) < 400:
            self.lmax = max(lext)   #한계값 설정, 차선 인식 오류 방지
            if self.lmax < 860:
                lavg = self.lmax

        if abs(min(rext)-self.rmin) < 400:
            self.rmin = min(rext)
            if self.rmin > 410:
                ravg = self.rmin
            
        cv2.circle(dst,(int(lavg),cy),6,(0,255,0),3) #왼쪽 차선 위치
        cv2.circle(dst,(int(ravg),cy),6,(0,255,0),3) #오른쪽 차선 위치

        center = (lavg + ravg)/2 #도로 중앙값 계산
        
        cv2.circle(dst,(int(center),135),6,(255,0,0),2) #도로 중앙
        cv2.circle(dst,(640,135),6,(0,0,255),3)    #차량 중앙
        cv2.line(dst, (0,135), (1280,135),(0,255,225), 2) #노란 선

        return center
    
    def Pub_Control_msg(self, center):
        camera_control = Float32MultiArray()
        camera_control.data = [0,0]
        steer = (center - 640)/5

        if steer < 2:
            speed = 13
        else:
            speed = 5
        camera_control.data[0] = speed
        camera_control.data[1] = steer

        self.pub.publish(camera_control)

    def Canny(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        canny = cv2.Canny(blur, 50, 200)
        
        return canny
if __name__ == '__main__':
    try:
        image_parser = Camera()
    except rospy.ROSInterruptException:
        pass
