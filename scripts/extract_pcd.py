import sys
import os
import shutil
import numpy as np
from math import pow, sqrt
from collections import deque

import rospy
from sensor_msgs.msg import CameraInfo, PointCloud2
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import PointStamped, do_transform_point
from geometry_msgs.msg import Point

import cv2
import open3d as o3d
from tf2_ros import TransformListener, TransformException, Buffer
from sensor_msgs.point_cloud2 import read_points
from std_msgs.msg import Header

TF_TIMEOUT_DURATION = 0.25

class Extractor:
    def __init__(self, pcd_topic, cam_info_topic, save_dir):
        self.pcd2_msg_queue = deque() # Queue of yet to be saved extracted point clouds.
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel() # Camera model to project point clouds to the camera frame.
        self.cam_model_obtained = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Make this node a subscriber to cam_info_topic.
        rospy.Subscriber(cam_info_topic, CameraInfo, self.cam_info_callback)
        # Make this node a subscriber to pcd_topic.
        rospy.Subscriber(pcd_topic, PointCloud2, self.point_cloud_callback)

        # Create (or empty) the point cloud directory.
        self.pcd_dir_path = os.path.join(save_dir, 'ros_extracted_pcd')
        if os.path.exists(self.pcd_dir_path):
            shutil.rmtree(self.pcd_dir_path)
        os.mkdir(self.pcd_dir_path)
        
        while not rospy.is_shutdown():
            # If point cloud message queue is not empty and camera model has been obtained:
            if self.pcd2_msg_queue and self.cam_model_obtained:
                pcd2_msg = self.pcd2_msg_queue.popleft()
                timestamp = pcd2_msg.header.stamp

                try:
                    # Get a temporally close transform for the camera frame.
                    # If none is obtained, skip this point cloud.
                    transform = self.tf_buffer.lookup_transform(self.cam_model.tfFrame(),
                                                            pcd2_msg.header.frame_id,
                                                            timestamp,
                                                            rospy.Duration(TF_TIMEOUT_DURATION))
                    if not transform:
                        return
                except TransformException as e:
                    rospy.logwarn("Transform exception: %s", e)
                    continue
                
                # Create an Open3D point cloud based on the received point cloud message.
                open3d_cloud = o3d.geometry.PointCloud()
                points = np.array(list(read_points(pcd2_msg, field_names=('x', 'y', 'z', 'power', 'doppler'))))
                try:
                    open3d_cloud.points = o3d.utility.Vector3dVector(points[:, :3])
                except RuntimeError as e:
                    rospy.logerr("Error setting points: %s", e)
                power_of_points = points[:, 3]
                doppler_of_points = points[:, 4]
                
                curr_point = PointStamped()
                curr_point.header.frame_id = pcd2_msg.header.frame_id
                curr_point.header.stamp = timestamp
                point_array = np.asarray(open3d_cloud.points)
                uv_points = []
                
                # Project each point in the point cloud onto the image plane.
                for i, point in enumerate(point_array):
                    curr_point.point.x, curr_point.point.y, curr_point.point.z = point
                    curr_point_proj = do_transform_point(curr_point, transform)
                    uv_rect = self.cam_model.project3dToPixel(np.array([curr_point_proj.point.x, curr_point_proj.point.y, curr_point_proj.point.z]))
                    
                    z_range = sqrt(pow(point[0], 2) + pow(point[1], 2) + pow(point[2], 2))
                    # Add point to uv_points only if z_range >= 5.
                    if z_range >= 5:
                        uv_points.append([uv_rect[0], uv_rect[1], z_range, power_of_points[i], doppler_of_points[i]])
                
                # Convert the list to a NumPy array.
                uv_points = np.array(uv_points)

                # Get the timestamp in milliseconds as a string with 9 decimal places
                # and with '_' as the decimal separator.
                timestamp_string = "{:.9f}".format(timestamp.to_sec()).replace('.', '_')

                pcd_to_save = o3d.geometry.PointCloud()
                pcd_to_save.points = o3d.utility.Vector3dVector(uv_points[:, :3]) # x, y, z
                # power and Doppler
                doppler_and_power = np.column_stack((power_of_points, doppler_of_points, np.zeros_like(power_of_points)))
                pcd_to_save.normals = o3d.utility.Vector3dVector(doppler_and_power)
                # Save the point cloud
                filename = timestamp_string + '.pcd'
                if o3d.io.write_point_cloud(os.path.join(self.pcd_dir_path, filename), pcd_to_save, write_ascii=True):
                    rospy.loginfo("Saved point cloud " + filename + ".")
                else:
                    rospy.logwarn("Warning: Failed to save point cloud " + filename + ".")

    def cam_info_callback(self, cam_info_msg):
        self.cam_model.fromCameraInfo(cam_info_msg)
        self.cam_model_obtained = True

    def point_cloud_callback(self, pcd2_msg):
        # Skip the point cloud if the queue is too long.
        if (len(self.pcd2_msg_queue)>=1):
            rospy.logwarn("WARNING: Point cloud queue is too long (length=" + str(len(self.pcd2_msg_queue)) +
                          "); skips this point cloud.")
            return
        # Otherwise, add it to the queue.
        else:
            self.pcd2_msg_queue.append(pcd2_msg)
    

if __name__ == '__main__':
    args = rospy.myargv(argv=sys.argv)[1:]

    # Make sure that exactly three argument was provided
    if len(args) != 3:
        rospy.logerr("ERROR: Wrong number of parameters provided.")
        sys.exit(1)

    rospy.init_node('extract_pcd')
    rospy.loginfo("ROS PCD extractor")
    extractor = Extractor(args[0], args[1], args[2])
