#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import open3d as o3d
import message_filters
import imutils

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        rgb_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(rgb_topic, Image, self.callback_rgb)
        depth_topic = 'camera/depth/image_raw'
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.callback_depth)
        # Node is subscribing to the rgb/image_raw topic
        # self.rgb_sub = message_filters.Subscriber('camera/rgb/image_raw', Image)

        # Node is subscribing to the /depth_to_rgb/image_raw topic
        # self.depth_sub = message_filters.Subscriber('camera/depth/image_raw', Image)

        
        # Allow up to one second to connection
        rospy.sleep(1)

    def callback_rgb(self, data):
        # Convert image to OpenCV format
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image_received = True
        self.rgb_image = rgb_image
    
    def callback_depth(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)
        self.depth_image = depth_image


    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            cv2.imwrite('rgb.jpg', self.rgb_image)
            cv2.imwrite('depth.jpg',cv2.normalize(self.depth_image,self.depth_image,0,255,cv2.NORM_MINMAX))
            o3d_image = o3d.geometry.Image(self.rgb_image)
            o3d_depth = o3d.geometry.Image(self.depth_image)
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_image, o3d_depth,
            convert_rgb_to_intensity = False)
            print(rgbd_image)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            # o3d.visualization.draw_geometries([pcd])
            o3d.io.write_point_cloud("point_cloud.xyz", pcd)
            # cv2.imwrite('rgbd.jpg', rgbd_image)
            return True
        else:
            return False

if __name__ == '__main__':

    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    camera = TakePhoto()

    # Take a photo

    # Use '_image_title' parameter from command line
    # Default value is 'photo.jpg'
    img_title = rospy.get_param('~image_title', 'depth.jpg')
    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")

    # Sleep to give the last log messages time to be sent
    rospy.sleep(1)
    