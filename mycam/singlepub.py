#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import time
import sys
sys.path.append('/home/lsky/anaconda3/envs/midas-py310/lib/python3.10/site-packages')

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera')
        self.left_pub = self.create_publisher(Image, '/camera/rgb/image_rect_color', 10)
        self.left_depth_pub = self.create_publisher(Image, '/camera/depth_registered/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_images)
        self.bridge = CvBridge()
        CamL_id = 2 # Camera ID for left camera
        #model_type = 'DPT_SwinV2_L_384'
        model_type = 'DPT_SwinV2_T_256'

        self.midas = torch.hub.load('/home/lsky/Dev/MiDaS/MiDaS', model_type,source='local') #pretrained=False
        #midas = torch.load('/home/lsky/Dev/MiDaS/MiDaS', model_type,source='local',)

        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()


        midas_transforms = torch.hub.load('/home/lsky/Dev/MiDaS/MiDaS','transforms',source='local')

        if model_type == " ":
            self.transform = midas_transforms.dpt_transform
        elif model_type == "DPT_SwinV2_L_384":
            self.transform = midas_transforms.swin384_transform
        elif model_type == "DPT_SwinV2_T_256":
            self.transform = midas_transforms.swin256_transform
        else:
            self.transform = midas_transforms.swin256_transform

        self.CamL= cv2.VideoCapture(CamL_id,cv2.CAP_V4L)
        self.left_camera_info = CameraInfo()

        self.left_camera_info.width = 640
        self.left_camera_info.height = 480


        self.left_camera_info.k = [1304.08998,    0.     ,  370.30178,
        0.     , 1303.06375,  304.07904,
        0.     ,    0.     ,    1.     ]


        self.left_camera_info.d = [-0.335327, 0.702490, 0.001413, 0.000687, 0.000000]


        self.left_camera_info.r = [ 0.99446913,  0.10400151,  0.01465748,
         -0.10413167,  0.99452793,  0.00841326,
         -0.01370228, -0.00989304,  0.99985718]


        self.left_camera_info.p = [1599.74481,    0.     ,  431.68785,    0.     ,
            0.     , 1599.74481,  258.3782 ,    0.     ,
            0.     ,    0.     ,    1.     ,    0.     ]


        #self.left_camera_info.header.frame_id = "base_link"


    def init_rectification_map(self, camera_info):
        K = np.array(camera_info.k).reshape((3, 3))
        D = np.array(camera_info.d)
        R = np.array(camera_info.r).reshape((3, 3))
        P = np.array(camera_info.p).reshape((3, 4))
        size = (camera_info.width, camera_info.height)
        map1, map2 = cv2.initUndistortRectifyMap(K, D, R, P, size, cv2.CV_32FC1)
        return map1, map2
    def publish_images(self):
        # Generate dummy images for demonstration
        retL, frameL = self.CamL.read()  # 카메라 영상 받기
        # Convert numpy array to ROS Image message

        now = self.get_clock().now().to_msg()

        h, w = frameL.shape[:2]
        kL = np.array([[1304.08998,    0.     ,  370.30178],
            [0.     , 1303.06375,  304.07904],
            [0.     ,    0.     ,    1.     ]])

        dL = np.array([-0.335327, 0.702490, 0.001413, 0.000687, 0.000000])
        new_K_left, roi_left = cv2.getOptimalNewCameraMatrix(kL, dL, (w, h), 1, (w, h))
        map1_left, map2_left = cv2.initUndistortRectifyMap(kL, dL, None, new_K_left, (w, h), 5)
        img_left_undistorted = cv2.remap(frameL, map1_left, map2_left, cv2.INTER_LINEAR)
        imgL = cv2.cvtColor(img_left_undistorted, cv2.COLOR_BGR2RGB)

        input_batch = self.transform(imgL).to(self.device)

        with torch.no_grad():
            prediction = self.midas(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=imgL.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()

        depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F)

        depth_map = (depth_map*255).astype(np.uint16)
        #depth_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)



        left_image_msg = self.bridge.cv2_to_imgmsg(img_left_undistorted, 'bgr8')
        left_image_msg.header.stamp = now
        left_image_msg.header.frame_id = 'camera_frame'
        # Publish images
        self.left_pub.publish(left_image_msg)

        self.left_camera_info.header.stamp = now
        self.left_camera_info.header.frame_id = 'camera_frame'
        self.left_info_pub.publish(self.left_camera_info)

        #check for the error
        left_image_depth_msg = self.bridge.cv2_to_imgmsg(depth_map, '16UC1')
        left_image_depth_msg.header.stamp = now
        left_image_depth_msg.header.frame_id = 'camera_frame'
        self.left_depth_pub.publish(left_image_depth_msg)

        #self.get_logger().info('Images published')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
