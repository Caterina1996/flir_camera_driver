#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
import os

class ImageExtractor:
    def __init__(self):
        rospy.init_node('image_extractor', anonymous=True)
        self.bag_file = '/home/uib/bagfiles/PLOME/data_peixos_andratx_MARCO/plome_camera/11_48_16/stereo_camera_images_2023-06-15-11-48-16_0.bag'
        self.output_folder = '/home/uib/PLOME/test_stereo/'
        self.image_count = 0

    def extract_images(self):
        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        bag = rosbag.Bag(self.bag_file, 'r')

        for topic, msg, t in bag.read_messages():
            if topic == '/stereo_ch3/left/image_color/compressed' or topic == '/stereo_ch3/right/image_color/compressed':
                try:
                    camera = topic.split("/")[2]
                    rospy.loginfo("Extracting image from {} camera...".format(camera))
                    image_msg = CompressedImage()
                    image_msg.deserialize(msg.data)

                    filename = os.path.join(self.output_folder, 'image_{}_{}.jpg'.format(camera, t.to_nsec()))
                    with open(filename, 'wb') as f:
                        f.write(image_msg.data)

                    self.image_count += 1
                    rospy.loginfo("Saved: {}".format(filename))
                except Exception as e:
                    rospy.logerr("Error processing image: {}".format(e))

        bag.close()
        rospy.loginfo("Image extraction complete. {} images saved.".format(self.image_count))

if __name__ == '__main__':
    try:
        image_extractor = ImageExtractor()
        image_extractor.extract_images()
    except rospy.ROSInterruptException:
        pass
