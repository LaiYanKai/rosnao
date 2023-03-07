Exposes some functionalities for the Capstone VSLAM projects in ROS.
Avoids complicated install and possible latency issues with other ROS implementations.

# Prequisites
1. Make sure you are on Ubuntu 20.04 LTS with ROS Noetic installed.
2. The robot used is Nao 6.

# First Time Setup
1. Download the SDK from [here](https://community-static.aldebaran.com/resources/2.8.5/naoqi-sdk-2.8.5.10-linux64.tar.gz), preferably to the home `~` folder.
2. Right-click the .tar.gz file and click **Extract Here**. Make sure the top extracted folder is called `naoqi-sdk-2.8.5.10-linux64` and contains a `toolchain.xml` (not in any nested folders).
3. Rename the top extracted folder from `naoqi-sdk-2.8.5.10-linux64` to `nao_sdk_cpp`.
4. Open a terminal, clone this repository, and go into the `rosnao` workspace folder.
```
cd ~
git clone https://github.com/LaiYanKai/rosnao.git
cd ~/rosnao
```
5. Make sure you have `pip` installed:
```
sudo apt install python3-pip
```
6. Assign permissions to all bash scripts in the workspace:
```
chmod +x *.sh
```
7. Open `init_qibuild.sh`  and make sure `SDK_DIR` is pointing to the correct path of the `nao_sdk_cpp` folder. It is if you downloaded the SDK into the home `~` folder. Save when done.
8. Install qibuild and configures it for this project. If prompted, install the qibuild if it is not installed, and choose `Unix Makefiles` for CMake Generator and `None` for IDE:
```
sh init_qibuild.sh
```

# Streaming the Grayscale Camera
## Publishing into Image Topic
1. Make sure the robot is started up, and connected to an access point via its wi-fi or an ethernet cable. Press the chest button to get its IP address.
2. In the `rosnao` workspace folder, open `run_cam.sh`. Edit the parameters, especially `NAO_IP`:
   * When editing the SH script, make sure there are no spaces around the `=` operator.
   * Fill in the IP address for the Nao Robot in `NAO_IP`.
   * If running Ubuntu on VirtualBox, make sure the network adapter is **Bridged Adapter**.
   * Set `RES=1` for QVGA (320\*240), or `RES=2` for VGA (640\*480).
   * Set `CAM=0` for the bottom camera, or `CAM=1` for the top camera.
   * The `TOPIC` is the ROS Image topic that the video feed will be published into.
   * The `FRAME` is `frame_id` field of the message in the topic. See [Header](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html) and [Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html).
   * Save and exit when done.


3. Run the camera and RViz.
```
cd ~/rosnao
./run_cam.sh
```
4. The feed is grayscale for OrbSLAM purposes.
## Viewing the Image Topic in RViz
DEPRECATED. RViz is already launched with run_cam.sh
1. In **another terminal**, launch RViz using:
```rviz```
2. In RViz, change the base frame to `world` (from `map`).
3. Add the `Image` display,
4. Then, change the topic in the `Image` display to whatever is in `TOPIC` IN `run_cam.sh`.
5. Make sure `run_cam.sh` is running in another terminal to see the camera stream in RViz.

# Walking the robot and turning its head
## Overview
A complete list of SDK functions, can be found [here](http://doc.aldebaran.com/2-8/naoqi/motion/control-walk-api.html#control-walk-api).
Only the bare minimum of the SDK is exposed in the class `Motion` in `src/rosnao_brige/include/motion.hpp`. 
1. `setAngle`: Can only turn the head to a specified yaw angle. 1.57 radians means look left 90 deg, -1.57 means look right. Angle is adjustable. Fractional max speed should be kept low to scan the environment. Can be blocking or unblocking.
2. `moveTo`: The robot can move to a pose in x, y, yaw, relative to current pose (i.e. in robot's frame). The speed of walking is already minimal to reduce head bobbing.
3. `wakeup`, `rest` and `moveInit` functions. (see comments in `test_motion.cpp`).

## Running and Testing
1. Make sure the robot is started up, and connected to an access point via its wi-fi or an ethernet cable. Press the chest button to get its IP address.

2. In the `rosnao` workspace folder, open `run_cam.sh` and edit the IP address of the NAO robot.

3. Run the motion:
```
cd ~/rosnao
./run_motion.sh
```
4. Modify `src/rosnao_bridge/src/test_motion.cpp` and try from there.

You can choose to export this package but must first add the `motion.hpp` into a library using `add_library` in `CMakeLists.txt` in order to use it.
