import tensorflow_models_base.venv_hack

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import tensorflow as tf
import numpy as np

import cv2
from cv_bridge2 import CvBridge, CvBridgeError


class ShoreFollowerDrive(Node):
    def __init__(self):
        super().__init__("classify")

        self.declare_parameter('~/model_dir', ".")
        self.declare_parameter('~/speed', 0.4)
        self.model_dir_ = self.get_parameter('~/model_dir').get_parameter_value().string_value
        self.speed_ = self.get_parameter('~/speed').get_parameter_value().double_value

        self.load_model()

        self.br = CvBridge()
        self.twist_pub_ = self.create_publisher(Twist, "~/twist", 1)
        self.image_sub_ = self.create_subscription(Image,"~/image", self.image_callback, 1)

    def load_model(self):
        print(self.model_dir_)
        self.model = tf.keras.models.load_model(self.model_dir_)

    def image_callback(self, data):
        raw = self.br.imgmsg_to_cv2(data,"bgr8")
        processed_ = np.expand_dims(cv2.resize(raw, (0,0), fx = 32.0/data.height, fy=32.0/data.width, interpolation=cv2.INTER_AREA), axis=0)
        self.twist_pub_.publish(self.image_to_rot(processed_))

    def image_to_rot(self, img):
        out = Twist()
        res = self.model(img, training=False)[0]
        assert(res.shape[0] == 3)
        print("%5.2f %5.2f %5.2f" %(res[0],res[1],res[2]))
        if res[0] > res[1] and res[0] > res[2]:
            out.linear.z = self.speed_
        elif res[2] > res[1] and res[2] > res[0]:
            out.linear.z = -self.speed_
        else: 
            out.linear.z = 0.0
        return out


def main(args=None):
    rclpy.init(args=args)

    drive = ShoreFollowerDrive()

    rclpy.spin(drive)

    face_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
