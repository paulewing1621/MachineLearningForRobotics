#!/usr/bin/python

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image,RegionOfInterest
from face_detect_msgs.msg import FaceDetections

from cv_bridge import CvBridge
import cv2

class FaceDetect(Node):
    def __init__(self):
        super().__init__("face_detect")
        self.min_size = (10, 10)
        self.image_scale = 2
        self.haar_scale = 1.2
        self.min_neighbors = 2
        self.haar_flags = 0
        self.display = True

        self.opencv_dir = '/usr/share/opencv4/haarcascades/'

        self.face_cascade = cv2.CascadeClassifier(self.opencv_dir + 'haarcascade_frontalface_default.xml')
        if self.face_cascade.empty():
            print("Could not find face cascade")
            sys.exit(-1)
        self.eye_cascade = cv2.CascadeClassifier(self.opencv_dir + 'haarcascade_eye.xml')
        if self.eye_cascade.empty():
            print("Could not find eye cascade")
            sys.exit(-1)
        self.br = CvBridge()
        self.declare_parameter('display', True)
        self.declare_parameter('eyes', True)
        self.display = self.get_parameter('display').get_parameter_value().bool_value
        self.detect_eyes = self.get_parameter('eyes').get_parameter_value().bool_value
        self.faces_pub = self.create_publisher(FaceDetections,"~/faces",10)
        self.image_pub = self.create_publisher(Image,"~/image_faces",10)
        self.sub = self.create_subscription(Image,"~/image", self.detect_and_draw, 1)

    def detect_and_draw(self,imgmsg):
        img = self.br.imgmsg_to_cv2(imgmsg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 3)
        face_msg = FaceDetections()
        face_msg.header = imgmsg.header
        for (x,y,w,h) in faces:
            roi = RegionOfInterest()
            roi.x_offset = int(x)
            roi.y_offset = int(y)
            roi.width = int(w)
            roi.height = int(h)
            roi.do_rectify = False
            face_msg.faces.append(roi)
            if self.display:
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            if self.detect_eyes:
                eyes = self.eye_cascade.detectMultiScale(roi_gray)
                if self.display:
                    for (x2,y2,w2,h2) in eyes:
                        cv2.rectangle(roi_color,(x2,y2),(x2+w2,y2+h2),(0,255,0),2)
        self.faces_pub.publish(face_msg)
        img_out = self.br.cv2_to_imgmsg(img, encoding="bgr8")
        img_out.header = imgmsg.header
        self.image_pub.publish(img_out)


        if self.display:
            cv2.imshow('img',img)
            cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)

    face_detect = FaceDetect()

    rclpy.spin(face_detect)

    face_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
