#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
import argparse
import os
import sys
# ROS Image message
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

"PYTHON 2 !!!!!!!!!!"

"python2 extract_disp.py --topic /stereo_down/scaled_x2/disparity --path_out ../test_save/ --node 1"

parser = argparse.ArgumentParser()
parser.add_argument('--topic', help='topic name')
parser.add_argument('--path_out', help='out folder path')
parser.add_argument('--node', default = 0, help='node number')
parsed_args = parser.parse_args(sys.argv[1:])

topic = parsed_args.topic
path_out = parsed_args.path_out
node = str(parsed_args.node)

if not os.path.exists(path_out):
    os.makedirs(path_out)

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        img = msg.image
        cv2_img = bridge.imgmsg_to_cv2(img)
    except CvBridgeError as e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        time = msg.header.stamp
        out = os.path.join(path_out, str(time) + '.jpeg')
        cv2.imwrite(out, cv2_img)
        rospy.sleep(1)

def main():
    rospy.init_node('image_listener_' + node)
    # Define your image topic
    # Set up your subscriber and define its callback
    rospy.Subscriber(topic, DisparityImage, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()