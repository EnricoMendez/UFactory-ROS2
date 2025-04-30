# Download ROS2
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
![image](https://github.com/user-attachments/assets/55497ea2-e78a-4cfc-bb91-ef937b1863d4)


Now add the ROS 2 GPG key with apt.

```
sudo apt update &&sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```



Then add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
![image](https://github.com/user-attachments/assets/dd47406c-d3f9-4448-882e-53ea5b82266b)

3. Install ROS packages
Update your apt repository caches after setting up the repositories.
```
sudo apt update
```

ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
```
sudo apt upgrade
```

Desktop Install (Recommended): ROS, RViz, demos, tutorials.
```
sudo apt install ros-humble-desktop
```

Install additional dependencies commonly used with ROS2 and init rosdep:

```
sudo apt install -y python3-argcomplete python3-colcon-common-extensions 
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

4. Test the ROS installation 

Set up your environment by sourcing the following file.
```
source /opt/ros/humble/setup.bash
```

If you installed ros-humble-desktop above you can try some examples.
In one terminal, source the setup file and then run a C++ talker:

```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
In another terminal source the setup file and then run a Python listener:
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```
![image](https://github.com/user-attachments/assets/089fbf6f-0ab8-4d82-95e7-5673892e7b75)
