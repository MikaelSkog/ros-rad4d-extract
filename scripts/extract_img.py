import sys
import os
import shutil
from collections import deque

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class Extractor:
    def __init__(self, img_topic, save_dir):
        self.img_msg_queue = deque() # Queue of image messages.
        self.bridge = CvBridge()

        # Make this node a subscriber to img_topic.
        rospy.Subscriber(img_topic, CompressedImage, self.callback)

        # Create (or empty) the image directory.
        img_dir_path = os.path.join(save_dir, 'ros_extracted_img')
        if os.path.exists(img_dir_path):
            shutil.rmtree(img_dir_path)
        os.mkdir(img_dir_path)

        while not rospy.is_shutdown():
            # If image queue is not empty:
            if self.img_msg_queue:
                # Get the image and timestamp of the first message in the queue.
                img_msg = self.img_msg_queue.popleft()
                img = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="bgr8") 
                timestamp = img_msg.header.stamp

                # Get the timestamp in milliseconds as a string with 9 decimal places
                # and with '_' as the decimal separator.
                timestamp_string = "{:.9f}".format(timestamp.to_sec()).replace('.', '_')

                # Save the image.
                filename = timestamp_string + '.jpg'
                if cv2.imwrite(os.path.join(img_dir_path, filename), img):
                    rospy.loginfo("Saved image " + filename + ".")
                else:
                    rospy.logwarn("Warning: Failed to save image " + filename + ".")
                

    def callback(self, received_msg):
        # Skip the image if the queue is too long.
        if (len(self.img_msg_queue)>=5):
            rospy.logwarn("WARNING: Image queue is too long (length=" + str(len(self.img_msg_queue)) +
                          "); skips this image.")
        # Otherwise, add it to the queue.
        else:
            self.img_msg_queue.append(received_msg)

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)[1:]

    # Make sure that exactly two argument was provided
    if len(args) != 2:
        print(len(args))
        rospy.logerr("ERROR: Wrong number of parameters provided.")
        sys.exit(1)

    rospy.init_node('extract_img')
    rospy.loginfo("ROS image extractor")
    extractor = Extractor(args[0], args[1])
    rospy.spin()