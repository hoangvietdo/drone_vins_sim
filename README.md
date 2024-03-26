# Drone Visual-Inertial Navigation System (VINS) Simulation

## 1. Overview
This is a drone simulation written in Matlab. The simulation employs a PID controller to guide a quadcopter along a given smooth trajectory and generate ground truth, IMU data, and monocular camera images using a pinhole model.
For instance, we simulate the drone flying in a circular pattern assuming that the camera is oriented downward, capturing randomly generated ground features.
The resulting dataset can be utilized to evaluate the performance of a visual-inertial navigation system (VINS).

![](/images/drone_vins.gif)

## 2. Prerequisites
- Matlab R2022b.
  
## 3. Usage
- The ```/IMU/drone.m``` script generates the ```/Datasets/drone_IMU.mat``` dataset includes IMU measurements (Forward-Right-Down), IMU specifications, and ground truth data (Forward-Left-Up).
- The ```/Camera/makeCamera.m``` script generates ground features based on the data from ```/Datasets/drone_IMU.mat```, records monocular camera data, and then combines all the information into the final dataset named ```/Datasets/drone_IMU_camera.mat```.
- Additionally, we provide a simplified version of an Extended Kalman Filter for Visual-Inertial Navigation System (EKF-VINS) algorithm to validate the dataset.

<p align="center">
<img src='/images/results.png' width='640' height='480'>
</p>

## 4. Credit / Acknowledgements
- This code was written at the [Intelligent Navigation and Control Systems Laboratory](https://sites.google.com/view/incsl), Sejong University, Seoul, Republic of Korea.
- This research was supported by the MSIT (Ministry of Science and ICT), Korea, under the ITRC (Information Technology Research Center) support program (IITP2021-2018-0-01423) supervised by the IITP (Institute for Information & Communications Technology Planning & Evaluation), and also be supported by the Korea Institute for Advancement of Technology (KIAT) grant funded by the Korea Government (MOTIE) (N0002431, The Competency Development Program for Industry Specialist).

## 5. Citation
If you find this work beneficial to your academic research, we would greatly appreciate it if you could reference our paper in your citations.
```bibtex
@INPROCEEDINGS{do2021amsckf,
  author={Do, Hoang Viet and Kim, Yong Hun and Kwon, Yeong Seo and Kang, San Hee and Kim, Hak Ju and Song, Jin Woo},
  booktitle={2021 21st International Conference on Control, Automation and Systems (ICCAS)}, 
  title={An Adaptive Approach based on Multi-State Constraint Kalman Filter for UAVs}, 
  year={2021},
  volume={},
  number={},
  pages={481-485},
  doi={10.23919/ICCAS52745.2021.9649897}
}
```

## 6. License
Our source code is released under the MIT license. If there are any issues in our source code please contact the author [hoangvietdo@sju.ac.kr](mailto:hoangvietdo@sju.ac.kr).
