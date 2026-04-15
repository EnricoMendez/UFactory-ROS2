## Prerequesites:

If you have dependencies problems, follow: https://github.com/EnricoMendez/UFactory-ROS2/blob/main/lectures/Visual_control_alternative

Install mediapipe [(soucre)](https://mediapipe-studio.webapps.google.com/studio/demo/gesture_recognizer), usb-cam ros pkg and v4l-utils.

```bash
sudo apt update
sudo apt install python3-pip  #Make sure you have pip
pip3 install mediapipe
sudo apt install ros-humble-usb-cam
sudo apt install v4l-utils  
```

Then, create a pkg with which we will work this sesion:

```bash
cd ~/dev_ws/src/
git clone https://github.com/EnricoMendez/visual_cobot.git
cd ~/dev_ws
colcon build --packages-select visual_cobot
```

## About Mediapipe

Mediapipe hand gesture model recognizes hand gestures and positions. Mediapipe can recognize 7 different gestures:


<img width="624" height="408" alt="image" src="https://github.com/user-attachments/assets/e8a18c8d-4e8a-4e43-95f1-e92266f0fe48" />


And calculate the positions of 21 markers of the hand:

<img width="875" height="329" alt="image" src="https://github.com/user-attachments/assets/5002c00c-11fb-4202-b027-c1ffa0630ad7" />

If you want to test mediapipe independently on your browser, go to: https://mediapipe-studio.webapps.google.com/demo/gesture_recognizer


[Python guide.](https://developers.google.com/mediapipe/solutions/vision/gesture_recognizer/python)

## Test your installation

First check which camera you'll be using, run:

```bash
v4l2-ctl --list-devices
```
<img width="603" height="108" alt="image" src="https://github.com/user-attachments/assets/49aa6ca7-58d0-4ed8-983f-5aabbfd841bc" />

Then run `ros2 launch visual_cobot gesture_recognition.launch.py video_device:='/dev/video0'` change `dev/video0` if needed to the device you will be using.
```bash
cd ~/dev_ws
source install/setup.bash
ros2 launch visual_cobot gesture_recognition.launch.py video_device:='/dev/video0'
```
## About services

To use services you need:
1. Create a client: `self.client = slef.create_client(<srv_type>, <srv_name>)
2. Wait for service:
   ```
   while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
   ```
3. Call the service: `self.future = self.client.call_async(self.req)`
4. Optional, wait for the service to finish: `rclpy.spin_until_future_complete(self, self.future)`

* You can check if service has finish with `self.client.done()`

How to get information from the services:

* List of services: `ros2 service list`
* About a service: `ros2 service type <srv_name>`
* About a service type: `ros2 interface show <type_name>`

## Try the code 

Let's review [this code](https://github.com/EnricoMendez/visual_cobot/blob/main/visual_cobot/visual_control.py)

To run the code use 3 terminals.
Terminal 1:
```bash
cd ~/dev_ws
source install/setup.bash
ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.0.11
```

Terminal 2:
```bash
cd ~/dev_ws
source install/setup.bash
ros2 launch visual_cobot gesture_recognition.launch.py video_device:='/dev/video0'
```

Terminal 3:
```bash
cd ~/dev_ws
source install/setup.bash
ros2 run visual_cobot visual_control
```

Now open and close your fist to control the robot's gripper.

## Activity
Now the challenge is to get to fully control the robot with signs

1. Modify `visual_control.py` so you can also turn off the gripper with a sign
2. Modify `visual_control.py` so you can fully control the robot movement and gripper, the objective is to be able to pick and place a block using only your hands.
