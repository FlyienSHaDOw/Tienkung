# **USD File of TienKung, a humanoid robot from Beijing Innovation Center of Humanoid Robotics**

# 1. Introduction

1.The world's first full-size fully electric-driven embodied intelligent robot capable of anthropomorphic running.  
3.Original Predictive Reinforcement limitation Learning method based on State Memory，more robust、more anthropomorphic、more generalized.  
4.Equipped with the largest scale,most dense, and most high-quality embodied intelligence dataset.  
5.Specific Parameters：  
- [x] Height:163cm   Weight:56kg   DoF:42  
- [x] High-precision six-dimensional force/torque sensors  
- [x] High-precision Inertial Measurement Unit (IMU)  
- [x] 3D Vision Sensor  
- [x] Operational computing performance per second: 550 trillion operations  
- [x] Battery: 48V 15AH  
- [x] Expandable computing power: 550TOPS  
- [x] Wireless communication、Wi-Fi + Bluetooth、5G-A network  
- [x] Open interfaces:Joint control interface、Sensor communication interface、Proprioceptive interface of the body  
- [x] Software capability expansion:Voice interaction、Intelligent grasping  

# 1.2 Structure
```
scene-simulator  
├──tienkung_main.py                    # standalone example script to control tienkung, 
│                                      # do PYTHON_PATH tienhung_main.py to run it.  
├──README.md                           # instructions of this project  
├──usd_model                           # USD file contained
│   ├──kienkung_with_hand_rendered.zip # unzip it in the same folder to get the usd file
│   └──texture                         # texture image of logo and camera
├──motion                              # scripts for path planning  
│   └──p2p_trajectory                  # generate smooth fifth-order trajectory
└──img                                 # leaflets  
```
# 2. Installation

## 2.1 System Requirements

Ubuntu 20.04 LTS Ubuntu 22.04 LTS
Python==3.10.13 (Recommended)

## 2.2 Dependencies
We recommend you to install the below dependencies in the default isaac sim python environment.
```shell
cd /path_to_isaac_sim_folder/ # e.g. ~/.local/share/ov/pkg/isaac_sim-2022.1.1
./python.sh -m pip install name_of_package_here
```

transforms3d==0.4.1  
numpy==1.21.0    
scipy==1.7.1  



