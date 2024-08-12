#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera_publisher')
        self.left_pub = self.create_publisher(Image, '/left/image_raw', 30)
        self.right_pub = self.create_publisher(Image, '/right/image_raw', 30)
        self.left_info_pub = self.create_publisher(CameraInfo, '/left/camera_info', 30)
        self.right_info_pub = self.create_publisher(CameraInfo, '/right/camera_info', 30)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_images)
        self.bridge = CvBridge()
        CamL_id = 4 # Camera ID for left camera
        CamR_id = 2# Camera ID for right camera
        
        self.CamL= cv2.VideoCapture(CamL_id,cv2.CAP_V4L)
        self.CamR= cv2.VideoCapture(CamR_id,cv2.CAP_V4L)
        self.left_camera_info = CameraInfo()
        self.right_camera_info = CameraInfo()


        self.left_camera_info.width = 640
        self.left_camera_info.height = 480
        self.right_camera_info.width = 640
        self.right_camera_info.height = 480

        self.left_camera_info.k = [1299.63546,    0.     ,  366.06103,
            0.     , 1302.11887,  317.00502,
            0.     ,    0.     ,    1.     ]
        self.right_camera_info.k = [1299.92168,    0.     ,  385.05671,
            0.     , 1303.1818 ,  238.01628,
            0.     ,    0.     ,    1.     ]

        self.left_camera_info.d = [-0.342942, 0.793768, 0.000835, -0.000075, 0.000000]
        self.right_camera_info.d = [-0.364047, 0.448864, 0.000880, 0.000654, 0.000000]
        self.left_camera_info.distortion_model = "plumb_bob"
        self.right_camera_info.distortion_model = "plumb_bob"
        self.left_camera_info.r = [ 0.99854144, -0.01189388,  0.05266435,
          0.01138366,  0.99988542,  0.00997758,
         -0.05277698, -0.00936351,  0.99856242]
        self.right_camera_info.r = [ 0.99961452, -0.00950937, -0.02608398,
          0.00925656,  0.99990918, -0.00979575,
          0.02617476,  0.00955053,  0.99961176]

        self.left_camera_info.p = [1631.24057,    0.     ,  360.59531,    0.     ,
            0.     , 1631.24057,  279.7433 ,    0.     ,
            0.     ,    0.     ,    1.     ,    0.     ]
        self.right_camera_info.p = [ 1631.24057,     0.     ,   360.59531, -1257.36508,
             0.     ,  1631.24057,   279.7433 ,     0.     ,
             0.     ,     0.     ,     1.     ,     0.     ]


    def publish_images(self):
        now = self.get_clock().now().to_msg()
        # Generate dummy images for demonstration
        retL, frameL = self.CamL.read()  # 카메라 영상 받기
        retR, frameR = self.CamR.read()  # 카메라 영상 받기
        # Convert numpy array to ROS Image message
        left_image_msg = self.bridge.cv2_to_imgmsg(frameL, 'bgr8')
        left_image_msg.header.stamp = now
        left_image_msg.header.frame_id = 'left_camera_frame'
        right_image_msg = self.bridge.cv2_to_imgmsg(frameR, 'bgr8')
        right_image_msg.header.stamp = now
        right_image_msg.header.frame_id = 'right_camera_frame'

        # Publish images
        self.left_pub.publish(left_image_msg)
        self.right_pub.publish(right_image_msg)
        self.left_camera_info.header.stamp = now
        self.left_camera_info.header.frame_id = 'left_camera_frame'
        self.left_info_pub.publish(self.left_camera_info)

        self.right_camera_info.header.stamp = now
        self.right_camera_info.header.frame_id = 'right_camera_frame'
        self.right_info_pub.publish(self.right_camera_info)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
