# UniO4-Aliengo

Implementation of the UNI-O4 framework on the Unitree Aliengo platform.
A modular framework for sim-to-real legged robotics, enabling robust policy learning, control, and real-world deployment.


## Unitree Aliengo SDK
https://github.com/unitreerobotics/unitree_legged_sdk/tree/Aliengo


## Offline Data Collection

```bash
cd unio4_data_collect
python go1_gym_deploy/scripts/deploy_policy.py --deploy_policy sim  # default: sim
```

## Folder Overview

| Folder                | Description                                                                                   |
|-----------------------|----------------------------------------------------------------------------------------------|
| **O2O_comparison_dataset** | Datasets for Offline-to-Online (O2O) evaluation, comparing simulation and real-world performance.|
| **aliengo**           | URDF models and configuration files for the Unitree Aliengo robot.   |
| **data_sim_policy**   | Real-world data from deploying simulation-trained policies on the robot.|
| **mocap**             | Motion capture tools for accurate estimation of robot velocity and pose.     |
| **pretraining**       |Scripts for simulation-based policy training and evaluation.|
| **realsense_t265_tx2**| Scripts to interface the Intel RealSense T265 tracking camera on Jetson TX2, publish its data over LCM, and broadcast pose/odometry for the Aliengo robot. See [realsense_t265_tx2/README.md](https://github.com/anubhav1772/UniO4-Aliengo/blob/main/realsense_t265_tx2/README.md) for details. |
| **unio4_data_collect**| Tools for offline data collection using the Aliengo platform, supporting policy deployment and logging.|
| **unio4_real**        | PPO-based reinforcement learning framework for real-world training on Aliengo.|
| **unitree_legged_sdk**| Unitree SDK and middleware (with LCM) for low-level communication, control, and state feedback.|


## Getting Started

- For offline data collection, see [`unio4_data_collect`](unio4_data_collect/).
- For RealSense T265 integration, see [`realsense_t265_tx2`](realsense_t265_tx2/README.md).
- For deploying trained policies to the real robot, see [`unio4_real`](unio4_real/).

Feel free to open issues or pull requests to contribute or report bugs.

