## Prerequisites 

To be able to operate the Ufactory Lite 6 arm via ROS2 it is necessary to download the xarm_ROS2 library ([source](https://github.com/xArm-Developer/xarm_ros2/blob/humble/ReadMe.md))
In these tutorials, the distribution of ROS2 'Humble' will be used, although the library is also compatible with foxy, galactic, rolling and Jazzy.

### 1. Download ROS2
To download ROS humble follow the instructions form [ROS2 doc](https://docs.ros.org/en/humble/index.html)

1. Set locale
Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```bash
locale  # check for UTF-8sudo apt update &&sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
exportLANG=en_US.UTF-8
locale  # verify settings
```

2. Set up source 
You will need to add the ROS 2 apt repository to your system.
First ensure that the Ubuntu Universe repository is enabled.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### 2. Install Moveit 2 and gazebo ros pkgs
In a new terminal after installing ROS Humble, execute:
```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-gazebo-ros-pkgs
```
### 3. Create a workspace 
On a new terminal create a new folder containing a src folder. You can name the folder however you want, here we'll call it "dev_ws".

```bash
mkdir -p dev_ws/src
```
## Xarm installation

### Obtain source code of "xarm_ros2" repository
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

### Additional requirements 

In order to later be able to run the simulations, it is needed to download the table from gazebo simulations. For this, open gazebo running in a terminal:

gazebo

Then the GUI will apear:

![image](https://github.com/user-attachments/assets/1eed49bf-a28d-4f2a-8aa9-43023444db8c)


Go to the insert tab and wait for http://model.gazebo… to load

![image](https://github.com/user-attachments/assets/01475be4-d828-4627-aa6a-e94c24020ec4)

Then expand the list and look for 'table'

![image](https://github.com/user-attachments/assets/04530f9c-40b9-41d5-a004-a89409c327e7)



Click on it and place it somewhere in the simulation, with this, the model will be downloaded.

![image](https://github.com/user-attachments/assets/65f1097d-53bb-400a-ab1a-3589be95e57f)


