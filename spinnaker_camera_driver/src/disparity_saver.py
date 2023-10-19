#!/usr/bin/env python

import rospy
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from message_filters import Subscriber, ApproximateTimeSynchronizer

class DisparitySaverROS:
    def __init__(self):
        rospy.init_node('disparity_saver', anonymous=True)
        self.bridge = CvBridge()

        # Define subscribers for disparity, left image, and right image
        self.disparity_sub = Subscriber('/stereo_ch3/scaled_x2/disparity/', DisparityImage)
        self.left_image_sub = Subscriber('/stereo_ch3/scaled_x2/left/image_rect_color', Image)
        self.right_image_sub = Subscriber('/stereo_ch3/scaled_x2/right/image_rect_color', Image)
        self.left_camera_info_sub = Subscriber('/stereo_ch3/scaled_x2/left/camera_info', CameraInfo)
        self.right_camera_info_sub = Subscriber('/stereo_ch3/scaled_x2/right/camera_info', CameraInfo)
        self.calibration_saved=False
        # Synchronize the callbacks based on the timestamps of messages using ApproximateTimeSynchronizer
        self.ts = ApproximateTimeSynchronizer([self.disparity_sub, self.left_image_sub, self.right_image_sub, self.left_camera_info_sub, self.right_camera_info_sub], queue_size=100, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.output_folder = rospy.get_param('~output_folder', '/home/uib/PLOME/test_stereo/')

    def callback(self, disparity_msg, left_image_msg, right_image_msg, left_camera_info_msg, right_camera_info_msg):

        disparity_image = self.bridge.imgmsg_to_cv2(disparity_msg.image, desired_encoding="passthrough")

        # Save the disparity image with the header_disparity as the name
        disparity_filename = os.path.join(self.output_folder, '{}_disparity.png'.format(disparity_msg.header.stamp))
        cv2.imwrite(disparity_filename, disparity_image)
        rospy.loginfo("Saved disparity image to {}".format(disparity_filename))

        # Save the left and right images
        left_image = self.bridge.imgmsg_to_cv2(left_image_msg, desired_encoding="passthrough")
        left_filename = os.path.join(self.output_folder, '{}_left_image.png'.format(disparity_msg.header.stamp))
        cv2.imwrite(left_filename, left_image)
        rospy.loginfo("Saved left image to {}".format(left_filename))

        right_image = self.bridge.imgmsg_to_cv2(right_image_msg, desired_encoding="passthrough")
        right_filename = os.path.join(self.output_folder, '{}_right_image.png'.format(disparity_msg.header.stamp))
        cv2.imwrite(right_filename, right_image)
        rospy.loginfo("Saved right image to {}".format(right_filename))

        # Save the desired fields to a text file
        if self.calibration_saved==False:

            info_filename = os.path.join(self.output_folder, 'disparity_info.yaml')

            with open(info_filename, 'w') as info_file:
                # save disparity params
                info_file.write("f: {}\n".format(disparity_msg.f))
                info_file.write("T: {}\n".format(disparity_msg.T))
                info_file.write("valid_window: {}\n".format(disparity_msg.valid_window))
                info_file.write("min_disparity: {}\n".format(disparity_msg.min_disparity))
                info_file.write("max_disparity: {}\n".format(disparity_msg.max_disparity))
                info_file.write("delta_d: {}\n".format(disparity_msg.delta_d))

            info_filename = os.path.join(self.output_folder, 'left_info.yaml')
            with open(info_filename, 'w') as info_file:
                # save left camera_info
                info_file.write(str(left_camera_info_msg))

            info_filename = os.path.join(self.output_folder, 'right_info.yaml')
            with open(info_filename, 'w') as info_file:
                # save right camera_info
                info_file.write(str(right_camera_info_msg))

            rospy.loginfo("Saved config")
            self.calibration_saved==True


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        saver = DisparitySaverROS()
        saver.run()
    except rospy.ROSInterruptException:
        pass