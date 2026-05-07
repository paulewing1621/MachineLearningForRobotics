#!/usr/bin/env python3

import tensorflow_models_base.venv_hack

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image

import tensorflow as tf
from tensorflow.keras import Model
import numpy as np

from sensor_msgs.msg import Image

import cv2
# cv_bridge2 is a local version of cv_bridge compiled with numpy 2
from cv_bridge2 import CvBridge, CvBridgeError


class FloorPlaneClassify(Node):
    def __init__(self):
        super().__init__("classify")

        self.declare_parameter('~/model_dir', ".")
        self.declare_parameter('~/thumb_size', 32)
        self.model_dir_ = self.get_parameter('~/model_dir').get_parameter_value().string_value
        self.ts_ = self.get_parameter('~/thumb_size').get_parameter_value().integer_value

        self.load_model()

        self.br = CvBridge()
        self.image_pub_ = self.create_publisher(Image,"~/image_label",1)
        self.image_sub_ = self.create_subscription(Image,"~/image", self.image_callback, 1)

    def load_model(self):
        # Loads the model
        self.model = tf.keras.models.load_model(self.model_dir_)

    def reshape_split(self, image, kernel_size):
        img_height, img_width, channels = image.shape
        tile_heigth, tile_width = kernel_size
        tiled_array = image.reshape(img_height // tile_heigth,
                                    tile_heigth,
                                    img_width // tile_width,
                                    tile_width,
                                    channels)
        tiled_array = tiled_array.swapaxes(1, 2)
        return tiled_array.reshape(-1, self.ts_, self.ts_, 3)

    def image_callback(self, data):
        # Gets the image, check that it is square and divisible by the thumbnail size
        assert(data.height == data.width)
        assert(data.height%self.ts_ == 0)
        # Convert to np array
        img = self.br.imgmsg_to_cv2(data,"bgr8")
        # Reshape the image as a batch of thumbnails (faster processing when using batches)
        #batch = np.reshape(np.array(np.split(np.array(np.split(img,self.ts_)),self.ts_)), [-1,self.ts_,self.ts_,3])
        batch = self.reshape_split(img, (self.ts_, self.ts_))
        # Calls the network
        checked = self.check_thumb(batch)
        # Transforms the array into low resolution image (on pixel per thumbnail)
        low_res = np.reshape(checked,[data.height//self.ts_,data.width//self.ts_,3])
        # Upsamples the predictions so they have the same size as the input image
        classified = cv2.resize(low_res,(0,0),fx=self.ts_,fy=self.ts_, interpolation=cv2.INTER_NEAREST).astype(np.uint8)
        overlay = cv2.addWeighted(img, 0.5, classified, 0.5, 0)
        # Publish the result
        enc = self.br.cv2_to_imgmsg(overlay,"rgb8")
        self.image_pub_.publish(enc)


    def check_thumb(self, batch):
        res = self.model(batch, training=False)
        print(res)
        threshold = 0.5
        array = np.zeros((res.shape[0],3))
        traversable = res[:,0]>threshold
        array[traversable] = [255,0,0]
        array[~traversable] = [0,255,0]
        return array


def main(args=None):
    rclpy.init(args=args)

    classify = FloorPlaneClassify()

    rclpy.spin(classify)

    face_detect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
