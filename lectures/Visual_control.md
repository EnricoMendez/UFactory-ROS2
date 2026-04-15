## Prerequesites:

Install mediapipe [(soucre)](https://mediapipe-studio.webapps.google.com/studio/demo/gesture_recognizer), usb-cam ros pkg and v4l-utils.

```bash
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

Then run `ros2 launch visual_cobot gesture_recognition.py video_device:='dev/video0'` change `dev/video0` if needed to the device you will be using.
```bash
ros2 launch visual_cobot gesture_recognition.py video_device:='dev/video0'
```

