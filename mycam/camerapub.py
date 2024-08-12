#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('stereo_camera')
        self.left_pub = self.create_publisher(Image, '/stereo_camera/left/image_rect_color', 10)
        self.right_pub = self.create_publisher(Image, '/stereo_camera/right/image_rect', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/stereo_camera/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/stereo_camera/right/camera_info', 10)
        timer_period = 0.1  # seconds
        CamL_id = 2 # Camera ID for left camera
        CamR_id = 0# Camera ID for right camera
        
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
        #self.left_camera_info.header.frame_id = "base_link"
        self.timer = self.create_timer(timer_period, self.publish_images)
        self.bridge = CvBridge()
        self.map1_left, self.map2_left = self.init_rectification_map(self.left_camera_info)
        self.map1_right, self.map2_right = self.init_rectification_map(self.right_camera_info)
        

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
        retR, frameR = self.CamR.read()  # 카메라 영상 받기
        # Convert numpy array to ROS Image message

        now = self.get_clock().now().to_msg()

        h, w = frameL.shape[:2]
        kL = np.array([[1299.63546,    0.     ,  366.06103],
            [0.     , 1302.11887,  317.00502],
            [0.     ,    0.     ,    1.     ]])
        kR = np.array([[1299.92168,    0.     ,  385.05671],
            [0.     , 1303.1818 ,  238.01628],
            [0.     ,    0.     ,    1.     ]])

        dL = np.array([-0.342942, 0.793768, 0.000835, -0.000075, 0.000000])
        dR = np.array([-0.364047, 0.448864, 0.000880, 0.000654, 0.000000])
        new_K_left, roi_left = cv2.getOptimalNewCameraMatrix(kL, dL, (w, h), 1, (w, h))
        new_K_right, roi_right = cv2.getOptimalNewCameraMatrix(kR, dR, (w, h), 1, (w, h))

        map1_left, map2_left = cv2.initUndistortRectifyMap(kL, dL, None, new_K_left, (w, h), 5)
        map1_right, map2_right = cv2.initUndistortRectifyMap(kR, dR, None, new_K_right, (w, h), 5)

        img_left_undistorted = cv2.remap(frameL, map1_left, map2_left, cv2.INTER_LINEAR)
        img_right_undistorted = cv2.remap(frameR, map1_right, map2_right, cv2.INTER_LINEAR)


            # 이미지 교정
        frameL_rect = cv2.remap(img_left_undistorted, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        frameR_rect = cv2.remap(img_right_undistorted, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
        grayR_rect = cv2.cvtColor(frameR_rect,cv2.COLOR_BGR2GRAY)
        left_image_msg = self.bridge.cv2_to_imgmsg(frameL_rect, 'bgr8')
        left_image_msg.header.stamp = now
        left_image_msg.header.frame_id = 'left_camera_frame'
        right_image_msg = self.bridge.cv2_to_imgmsg(grayR_rect, 'mono8')
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

        #self.get_logger().info('Images published')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
