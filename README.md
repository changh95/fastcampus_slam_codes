# fastcampus_slam_codes

This repository contains code exercises for the following lecture series provided by @changh95 at FastCampus:

- ['Computer Vision, LiDAR processing, and Sensor Fusion for Autonomous Driving']
- ['SLAM Zero-to-Hero series for Physical AI and 3D Computer Vision']

> Actively reworking the repository now. Stay tuned, because A LOT OF NEW TUTORIALS are on the way!

 </b>

## Zero-to-Hero SLAM lectures for Physical AI and 3D Computer Vision

![](./SLAM_zero_to_hero/title.png)

The course can be found [here](https://fastcampus.co.kr/data_online_slam).

The course content is essentially a superset of 'Computer Vision, LiDAR processing, and Sensor Fusion for Autonomous Driving', but with a more general focus within robotics, drones, AR/VR, autonomous driving. 

Content will be updated soon.

## Computer Vision, LiDAR processing, and Sensor Fusion for Autonomous Driving

![](./Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/title.png)

The course can be found [here](https://fastcampus.co.kr/data_online_autovehicle).

This course contains the following contents:

- Chapter 1: Introduction to SLAM
  - 1.1 Lecture introduction
  - 1.2 What is SLAM?
  - 1.3 Hardware for SLAM
  - 1.4 Types of SLAM
  - 1.5 Applications of SLAM
  - 1.6 Before we begin...
  - 1.7 [Basic C++ / CMake](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/1_7)
- Chapter 2: Introduction 3D Spaces
  - 2.1 3D rotation and translation
  - 2.2 [3D rotation and translation, using Eigen library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/2_2)
  - 2.3 Homogeneous coordinates
  - 2.4 Lie Group
  - 2.5 Basic Lie algebra
  - 2.6 [Lie Group and Lie algebra, using Sophus library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/2_6)
  - 2.7 How cameras work
  - 2.8 How LiDARs work
- Chapter 3: Image processing
  - 3.1 Local feature extraction & matching
  - 3.2 [Local feature extraction & matching, using OpenCV library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/3_2)
  - 3.3 [Superpoint and Superglue, using C++ and TensorRT](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/3_3)
  - 3.4 Global feature extraction
  - 3.5 [Bag of Visual Words, using DBoW2 library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/3_5)
  - 3.6 [Learning-based global feature extraction, using PyTorch and Tensorflow libraries](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/3_6)
  - 3.7 Feature tracking
  - 3.8 [Optical flow, using OpenCV library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/3_8)
- Chapter 4: Point cloud processing
  - 4.1 Introduction to point cloud processing
  - 4.2 [Introduction to point cloud processing, using PCL library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/4_2)
  - 4.3 Point cloud pre-processing
  - 4.4 [Point cloud pre-processing, using PCL library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/4_4)
  - 4.5 Iterative closest point
  - 4.6 [Iterative closest point, using PCL library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/4_6)
  - 4.7 Advanced ICP methods
  - 4.8 [Advanced ICP methods (G-ICP, NDT, TEASER++, KISS-ICP), using PCL library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/4_8)
  - 4.9 [Octree, Octomap, Bonxai, using PCL/Octomap/Bonxai libraries](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/4_9)
- Chapter 5: Multiple view geometry
  - 5.1 Epipolar geometry
  - 5.2 [Essential and Fundamental matrix estimation, using OpenCV library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/5_2)
  - 5.3 Homography
  - 5.4 [Bird's eye view (BEV) projection, using OpenCV library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/5_4)
  - 5.5 [Simple monocular visual odometry, using OpenCV library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/5_5)
  - 5.6 Triangulation
  - 5.7 [Triangulation, using OpenCV library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/5_7)
  - 5.8 Perspective-n-Points (PnP) and Direct Linear Transform (DLT)
  - 5.9 [Fiducial marker tracking, using OpenCV library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/5_9)
  - 5.10 RANSAC
  - 5.11 Advanced RANSAC methods (USAC)
  - 5.12 [RANSAC and USAC, using OpenCV and RansacLib libraries](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/5_12)
  - 5.13 Graph-based SLAM
  - 5.14 Least squares
  - 5.15 Schur complement
  - 5.16 Bundle adjustment
  - 5.17 [Bundle adjustment, using Ceres-Solver library](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/5_17)
- Chapter 6: Visual-SLAM
  - 6.1 Overview of feature-based VSLAM
  - 6.2 Overview of direct VSLAM
  - 6.3 Overview of visual-inertial odometry (VIO)
  - 6.4 Spatial AI
  - 6.5 [ORB-SLAM2](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/orb_slam2), [ORB-SLAM3](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/orb_slam3)
  - 6.6 [DynaVINS](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/dynavins)
  - 6.7 [CubeSLAM](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/cubeslam)
- Chapter 7: LiDAR SLAM
  - 7.1 Overview of 2D LiDAR SLAM
  - 7.2 Overview of 3D LiDAR SLAM and LiDAR-inertial odometry
  - 7.3 [HDL-Graph-SLAM](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/hdl_graph_slam)
  - 7.4 [KISS-ICP](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/kiss_icp)
  - 7.5 [SHINE-Mapping](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/shine_mapping)
- Chapter 8: CI/CD for SLAM
  - 8.1 TDD and tests
  - 8.2 CI/CD
  - 8.3 CI agents
  - 8.4 CI/CD for Python SLAM projects
  - 8.5 CI/CD for C++ SLAM projects
- Final projects:
  - [DSP-SLAM](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/dsp_slam)
  - [Suma++](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/suma_plus_plus)
  - [Cartographer-KITTI](Computer_Vision_LiDAR_Processing_and_Sensor_Fusion_for_Autonomous_Driving/cartographer)


