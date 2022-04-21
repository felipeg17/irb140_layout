#!/usr/bin/env python

import rospy
import message_filters # To Achieve Multiple subscriber
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
import imutils

class Kinect_Node(object):
    def __init__(self):
      
        #use CvBridge to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Node is subscribing to the rgb/image_raw topic
        self.rgb_sub = message_filters.Subscriber('camera/rgb/image_raw', Image)

        # Node is subscribing to the /depth_to_rgb/image_raw topic
        self.depth_sub = message_filters.Subscriber('camera/depth/image_raw', Image)


    def callback(self, ros_rgb, ros_depth):
        
        #convert ROS image message to OpenCV image 
        depth_frame = self.br.imgmsg_to_cv2(ros_depth, desired_encoding="32FC1")
        rgb_frame = self.br.imgmsg_to_cv2(ros_rgb, "bgr8")
        # cv2.imwrite('depth.jpg',cv2.normalize(depth_frame,depth_frame,0,255,cv2.NORM_MINMAX))
        #resize image
        depth_frame = imutils.resize(depth_frame, width=640)
        rgb_frame = imutils.resize(rgb_frame, width=640)
        
        depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_GRAY2BGR)
        #rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGRA2BGR)
        
        #If we want to stablize, don't use imshow and waitKey(1). Use a publisher to send the data out and use rqt_gui to see the frame.
        cv2.imshow("depth camera", depth_frame)
        cv2.imshow("rgb camera", rgb_frame)
        
        cv2.waitKey(1)
            
def main():

    my_node = Kinect_Node()
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name. 
    rospy.init_node("kinect_sub_py", anonymous=True)

    ts = message_filters.ApproximateTimeSynchronizer([my_node.rgb_sub, my_node.depth_sub], 10, 0.1)
    ts.registerCallback(my_node.callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    main()