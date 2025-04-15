<!--Read data from the Intel RealSense T265 tracking camera mounted on the AlienGo robot 
    and publish it using the LCM (Lightweight Communications and Marshalling) library. -->

## Intel RealSense T265 Tracking Camera data broadcast via LCM
The RealSense T265 sensor is employed to measure the robot’s speed along the x, y, and yaw axes for reward calculation.

Intel® RealSense™ SDK 2.0 (for T265 Tracking Camera): https://github.com/IntelRealSense/librealsense/tree/v2.50.0?tab=readme-ov-file

### Terminal 1 
```bash
# Remotely log in to Aliengo TX2 via SSH
ssh unitree@192.168.123.12
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1
# ip link show
sudo ifconfig eth0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0
./camera_lcm_msgs
```

### Terminal 2
```bash
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1
# modify enp3s0f1 with your own value (ip link show)
sudo ifconfig enp3s0f1 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp3s0f1
cd unitree_legged_sdk/build
sudo ./lcm_position
```
