## Prerequisites 

To be able to operate the Ufactory Lite 6 arm via ROS2, it is necessary to download the xarm_ROS2 library ([source](https://github.com/xArm-Developer/xarm_ros2/blob/humble/ReadMe.md))
The ROS2 'Humble' distribution will be used in these tutorials, although the library is also compatible with foxy, galactic, rolling, and Jazzy.

### 1. Download ROS2
To download ROS Humble follow the instructions from [ROS2 doc](https://docs.ros.org/en/humble/index.html)

Or follow the [ROS2 installation tutorial](https://github.com/EnricoMendez/UFactory-ROS2/blob/main/tutorials/ROS2_Humble_installation.md) of this repo.

### 2. Install Moveit 2
In a new terminal after installing ROS Humble, execute:
```bash
sudo apt install ros-humble-moveit
```
### 3. Create a workspace 
On a new terminal create a new folder containing a src folder. You can name the folder however you want, here we'll call it "dev_ws".

```bash
mkdir -p dev_ws/src
```
## Xarm installation

### Obtain the source code of "xarm_ros2" repository
Inside your ROS workspace, execute:

```bash
cd ~/dev_ws/src/
git clone https://github.com/xArm-Developer/xarm_ros2.git --recursive -b humble
```
> Note: if needed, install git running: `sudo apt install git`

Update the repository

```bash
cd ~/dev_ws/src/xarm_ros2
git pull
git submodule sync
git submodule update --init --remote
```

Install dependencies

```bash
source /opt/ros/humble/setup.bash
cd ~/dev_ws/src/
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```



Build xarm_ros2

```bash
cd ~/dev_ws/
colcon build  
```


## Using Xarm
There are different ways you can use xarm:

1. Control real robot
2. Moveit control

### 1. Control real robot

To control the U Lite 6, the first thing to do is to launch the ufactory driver node, adding as an argument the IP of the robot you are working with, with this command:

```ros2 launch xarm_api lite6_driver.launch.py robot_ip:=<robot_ip>```
> Example:
```ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.0.11```
> This will start the communication between ROS and the robot. 

Then, to enable the robot's joints, you must run:

```ros2 service call /ufactory/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"```

Then, to move the robot using Xarm, you must set the robot in mode 0 and state 0.

```ros2 service call /ufactory/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"```

```ros2 service call /ufactory/set_state xarm_msgs/srv/SetInt16 "{data: 0}"```

Finally, you can now move the robot with linear motion with: 

```ros2 service call /ufactory/set_position xarm_msgs/srv/MoveCartesian "{pose: [x,y,z,y,p,r], speed: 50, acc: 500, mvtime: 0}" ```

Or with joint moves using: 

```ros2 service call /ufactory/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [j1,j2,j3,j4,j5,j6], speed: 0.35, acc: 10, mvtime: 0}"```

Here is a complete example of what can be done to move the robot

#### Moving robot example

launch ufactory_driver_node, in one terminal run the UFactory driver node:
```
cd ~/dev_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xarm_api lite6_driver.launch.py robot_ip:=<robot_ip>
```
> Change robot_ip to your own robot ip

![image](https://github.com/user-attachments/assets/63bf96bf-1157-48ef-bfaf-f5b19cc9df66)

Then, on a new terminal:
```
cd ~/dev_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then make sure to release the stop button and enable all joints:
```
ros2 service call /ufactory/motion_enable xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"
```
![image](https://github.com/user-attachments/assets/25409f22-4e9d-4c92-a8ac-a5ce525265a2)

Set proper mode (0) and state (0)
```
ros2 service call /ufactory/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
ros2 service call /ufactory/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```
![image](https://github.com/user-attachments/assets/919036a5-4fa0-4af1-9ce8-f7fa66b030b8)

Cartesian linear motion: (unit: mm, rad)
```
ros2 service call /ufactory/set_position xarm_msgs/srv/MoveCartesian "{pose: [250, 0, 250, 3.14, 0, 0], speed: 50, acc: 500, mvtime: 0}"
``` 
![image](https://github.com/user-attachments/assets/ff48948f-fa42-4e5b-bd24-5a47746e8e09)

Joint motion: (unit: rad)
```
ros2 service call /ufactory/set_servo_angle xarm_msgs/srv/MoveJoint "{angles: [-0.58, 0, 0, 0, 0, 0], speed: 0.35, acc: 10, mvtime: 0}"
```
![image](https://github.com/user-attachments/assets/76d80a33-d736-4877-86dc-38a79e9e6472)


> **Note**: please study the meanings of [Mode](https://github.com/xArm-Developer/xarm_ros/tree/master?tab=readme-ov-file#61-mode-explanation), State and available motion instructions before testing on the real robot. Please note the services provided by xArm series and Lite 6 have different namespaces.

### 2. Moveit control
To control the robot with move it you can control a simulation with:

```
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py [add_gripper:=true]
```
![image](https://github.com/user-attachments/assets/2eb49e22-adef-4b37-99cd-cf6c010b56f4)

Now moving the robot model in rviz and configuring the planning time, you can plan and execute trajectory.
> Note: be sure to set an appropriate planning time for your trajectory, if too short it won't make it.

To control the real robot launch:

```
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.0.31 [add_gripper:=true]
```
> Change robot_ip to your own robot IP

### Mode explanation 
> Taken from [source](https://github.com/xArm-Developer/xarm_ros/tree/master?tab=readme-ov-file#61-mode-explanation)

***Mode 0*** : xArm controller (Position) mode.  
***Mode 1*** : External trajectory planner (position) mode.  
***Mode 2*** : Free-Drive (zero gravity) mode.  
***Mode 3*** : Reserved.  
***Mode 4*** : Joint velocity control mode.  
***Mode 5*** : Cartesian velocity control mode.  
***Mode 6*** : Joint space online planning mode. (Firmware >= v1.10.0)  
***Mode 7*** : Cartesian space online planning mode. (Firmware >= v1.11.0)  

***Mode 0*** is the default when system initiates, and when error occurs(collision, overload, overspeed, etc), system will automatically switch to Mode 0. Also, all the motion plan services in [xarm_api](./xarm_api/) package or the [SDK](https://github.com/xArm-Developer/xArm-Python-SDK) motion functions demand xArm to operate in Mode 0. ***Mode 1*** is for external trajectory planner like Moveit! to bypass the integrated xArm planner to control the robot. ***Mode 2*** is to enable free-drive operation, robot will enter Gravity compensated mode, however, be sure the mounting direction and payload are properly configured before setting to mode 2. ***Mode 4*** is to control arm velocity in joint space. ***Mode 5*** is to control arm (linear) velocity in Cartesian space. ***Mode 6 and 7*** are for dynamic realtime response to newly generated joint or Cartesian target respectively, with automatic speed-continuous trajectoty re-planning.

#### Proper way to change modes:  
If collision or other error happens during the execution of a Moveit! planned trajectory, Mode will automatically switch from 1 to default mode 0 for safety purpose, and robot state will change to 4 (error state). The robot will not be able to execute any Moveit command again unless the mode is set back to 1. The following are the steps to switch back and enable Moveit control again:  

(1) Make sure the objects causing the collision are removed.  
(2) clear the error:  
```bash
rosservice call /xarm/clear_err
```
(3) switch to the desired mode (Mode 2 for example), and set state to 0 for ready:
```bash
rosservice call /xarm/set_mode 2

rosservice call /xarm/set_state 0
```
