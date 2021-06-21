# Common function

Description: This repo contains some common functions that we will use frequently and all function is written by myself and some part of function call other libraries. The detail parts of this repo contains showing as following list:

- **CPP**

  - FileOp.cc: file operations like read txt file
  - HandEyeNonlinear: nonlinear optimization method to solve hand-eye calibration problem using ceres library.
  - RANSAC: ransac implementation using PCL libraries( TODO: implement it without using any third-party libraries)
  - Visualize: Project point cloud into 2D RGB image, visualize point cloud using pcl visualizer library, etc.
  - PCDEdgeLine:  To extract edge points from raw point cloud but in this function the point cloud structure is restricted to 16 beam LiDAR.(TODO: simplify algorithm and make it clearer)

- **ROS**

  - BagReader.cc: Read image and pcd from .bag files 

- **Python**

  





