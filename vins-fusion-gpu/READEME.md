# VINS-Fusion-gpu
This repository is a version of VINS-Fusion with GPU acceleration. It can run on Nvidia TX2 in real-time. 

## 1. Prerequisites  
The essential software environment is same as VINS-Fusion. Besides, it requires OpenCV cuda version.(Only test it on OpenCV 4.x).
## 2. Usage
### 2.1 Change the opencv path in the CMakeLists
In /vins_estimator/CMakeLists.txt, change Line 20 to your path.  
In /loop_fusion/CmakeLists.txt, change Line 19 to your path.
### 2.2 Change the acceleration parameters as you need.
In the config file, there are two parameters for gpu acceleration.  
use_gpu: 0 for off, 1 for on  
use_gpu_acc_flow:  0 for off, 1 for on  
If your GPU resources is limitted or you want to use GPU for other computaion. You can set  
use_gpu: 1  
use_gpu_acc_flow: 0  
If your other application do not require much GPU resources, I recommanded you to set  
use_gpu: 1  
use_gpu_acc_flow: 1  
According to my test, on TX2 if you set this two parameters to 1 at the same time, the GPU usage is about 20%.

# VINS-Fusion
## An optimization-based multi-sensor state estimator

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/vins_logo.png" width = 55% height = 55% div align=left />
<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti.png" width = 34% height = 34% div align=center />

VINS-Fusion is an optimization-based multi-sensor state estimator, which achieves accurate self-localization for autonomous applications (drones, cars, and AR/VR). VINS-Fusion is an extension of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), which supports multiple visual-inertial sensor types (mono camera + IMU, stereo cameras + IMU, even stereo cameras only). We also show a toy example of fusing VINS with GPS. 
**Features:**
- multiple sensors support (stereo cameras / mono camera+IMU / stereo cameras+IMU)
- online spatial calibration (transformation between camera and IMU)
- online temporal calibration (time offset between camera and IMU)
- visual loop closure

<img src="https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/support_files/image/kitti_rank.png" width = 80% height = 80% />

We are the **top** open-sourced stereo algorithm on [KITTI Odometry Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) (12.Jan.2019).

**Authors:** [Tong Qin](http://www.qintonguav.com), Shaozu Cao, Jie Pan, [Peiliang Li](https://peiliangli.github.io/), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [Aerial Robotics Group](http://uav.ust.hk/), [HKUST](https://www.ust.hk/)