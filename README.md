# ros-rad4d-extract
Python 3 scripts for ROS nodes that extract and then save images and point clouds published on ROS. The saved images and PCD files are to be used as input for tmva4d-compile-dataset.

## Getting Started
### Prerequisites
In order to run the ROS nodes, make sure that the following are installed in your Python environment: NumPy, OpenCV (CV2), Open3D. To install the packages using PIP:
```
pip3 install numpy opencv-python open3d
```

### Installation
1. Clone the repo into the _src_ folder of your ROS Catkin workspace:
   ```
   git clone https://github.com/MikaelSkog/ros-rad4d-extract.git
   ```
2. Build this package in your Catkin workspace. In your workspace directory, execute
   ```
   catkin_make
   ```

## Usage
### Image Extractor
With ROS running, ensure that the desired compressed images are published on ROS. To run the image extractor node, execute the following command:
```
rosrun rad4d_extract extract_img.py </compressed/image/topic> <image/destination/directory>
```
### Point Cloud Extractor
With ROS running, ensure that the desired point clouds, as well as camera info, are published on ROS. To run the point cloud extractor node, execute the following command:
```
rosrun rad4d_extract extract_pcd.py </pointcloud2/topic> </camera/info/topic> <pcd/destination/directory>
```
If you see warnings stating that the point cloud queue is too long (and you are publishing the point clouds by replaying ROS bags), consider lowering the playback speed until the warnings disappear. Note that the camera info topic should correspond to the camera whose images are published on the image topic. Also note that transforms must exist from the point clouds to the camera frame.
