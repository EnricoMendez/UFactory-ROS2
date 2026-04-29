# Pose estimation and block pick

## Prerequisites

Before starting this session, you should already have:

* ROS2 Humble installed.
* `xarm_ros2` installed and built in your workspace.
* The UFactory Lite 6 connected to the computer.
* The RealSense camera connected and working.
* The Lite 6 gripper services enabled in `xarm_params.yaml`:

```yaml
open_lite6_gripper: true
close_lite6_gripper: true
stop_lite6_gripper: true
```

> If you have not enabled these parameters, follow the previous session instructions before continuing.

## 1. Download the pose estimation package

Inside your ROS workspace, clone the package that estimates the block position.

```bash
cd ~/dev_ws/src/
git clone https://github.com/EnricoMendez/pose_estimation_cobot.git
```

Then install dependencies and build the package.

```bash
cd ~/dev_ws/
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --packages-select pose_estimation_cobot
source install/setup.bash
```

## 2. Run the full pose estimation stack

This launch file starts:

* RealSense camera.
* Lite 6 control stack.
* `segmentation` node.
* `object_pose_estimator` node.
* RViz.
* Static transform between the end effector and the camera.

Run:

```bash
cd ~/dev_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pose_estimation_cobot rviz_pose_estimation.launch.py robot_ip:=<robot_ip>
```

Example:

```bash
ros2 launch pose_estimation_cobot rviz_pose_estimation.launch.py robot_ip:=192.168.0.12
```

> Change `<robot_ip>` to the IP address of your own robot.

## 3. What the code does

The package has two main nodes.

### Segmentation node

The `segmentation` node subscribes to the RealSense color image:

```bash
/camera/camera/color/image_rect_raw
```

It converts the image from BGR to HSV and segments the red block using two HSV ranges, because red wraps around the HSV hue limits.

It publishes:

```bash
/segmented_img
/segmented_img_overlay
```

The topic `/segmented_img` is a binary image where white pixels correspond to the detected block.

### Object pose estimator node

The `object_pose_estimator` node synchronizes:

```bash
/camera/camera/color/image_rect_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
/segmented_img
```

Then it uses the camera intrinsics and the depth image to project each valid pixel into 3D:

```python
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = depth
```

After this, it keeps only the points that belong to the segmented block and calculates the centroid of each detected blob.

Finally, it transforms the centroid from the camera frame to the robot base frame using TF.

The most important output topic is:

```bash
/camera/camera/depth/color/points_segmented_centroid
```

This topic publishes a `geometry_msgs/msg/PointStamped` with the block position in the robot base frame.

## 4. Check the outputs

In a new terminal run:

```bash
cd ~/dev_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list
```

You should see these topics:

```bash
/segmented_img
/segmented_img_overlay
/camera/camera/depth/color/points_from_images
/camera/camera/depth/color/points_segmented
/camera/camera/depth/color/points_segmented_centroid
/camera/camera/depth/color/points_segmented_centroids
/camera/camera/depth/color/points_segmented_centroid_markers
```

To check the estimated block position, run:

```bash
ros2 topic echo /camera/camera/depth/color/points_segmented_centroid --once
```

You should see something similar to:

```bash
header:
  frame_id: link_base
point:
  x: 0.25
  y: 0.03
  z: 0.08
```

> The position is in meters. The Lite 6 Cartesian service uses millimeters.

## 5. Activity: pick the block

Now you will create a new ROS2 node that uses the estimated position of the block and moves the robot to pick it.

The node must:

1. Subscribe to `/camera/camera/depth/color/points_segmented_centroid`.
2. Store the latest valid centroid.
3. Convert the position from meters to millimeters.
4. Move the robot to a safe home position.
5. Open the gripper.
6. Move above the block.
7. Move down to the grasp position.
8. Close the gripper.
9. Stop the gripper.
10. Lift the block.
11. Return to the home position.

## 6. Create your package

Create a new package for this activity.

```bash
cd ~/dev_ws/src/
ros2 pkg create pick_block_pkg --build-type ament_python --node-name pick_block
```

Build the package.

```bash
cd ~/dev_ws/
colcon build --packages-select pick_block_pkg
source install/setup.bash
```

## 7. Suggested code structure

Open the file:

```bash
nano ~/dev_ws/src/pick_block_pkg/pick_block_pkg/pick_block.py
```

Your code should include:

```python
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from xarm_msgs.srv import MoveCartesian, SetInt16ById, SetInt16, Call
```

Create a node that subscribes to the centroid topic:

```python
self.centroid_sub = self.create_subscription(
    PointStamped,
    '/camera/camera/depth/color/points_segmented_centroid',
    self.centroid_callback,
    10
)
```

Store the latest centroid:

```python
def centroid_callback(self, msg):
    self.last_centroid = msg.point
    self.get_logger().info(
        f'Block position: x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f}',
        throttle_duration_sec=1.0
    )
```

Create clients for the robot services:

```python
self.motion_enable_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
self.move_client = self.create_client(MoveCartesian, '/ufactory/set_position')
self.open_gripper_client = self.create_client(Call, '/ufactory/open_lite6_gripper')
self.close_gripper_client = self.create_client(Call, '/ufactory/close_lite6_gripper')
self.stop_gripper_client = self.create_client(Call, '/ufactory/stop_lite6_gripper')
```

Use a Cartesian pose with this format:

```python
pose = [x_mm, y_mm, z_mm, roll, pitch, yaw]
```

For this activity, you can start with a vertical tool orientation:

```python
roll = 3.14
pitch = 0.0
yaw = 0.0
```

> Adjust the orientation if your real gripper is not aligned with the block.

## 8. Pick sequence

Use this sequence as your reference:

```python
home = [250.0, 0.0, 120.0, 3.14, 0.0, 0.0]

x = self.last_centroid.x * 1000.0
y = self.last_centroid.y * 1000.0
z = self.last_centroid.z * 1000.0

pregrasp = [x, y, z + 50.0, 3.14, 0.0, 0.0]
grasp = [x, y, z + grasp_offset, 3.14, 0.0, 0.0]
lift = [x, y, z + 100.0, 3.14, 0.0, 0.0]

self.send_move(home)
self.open_gripper()
self.stop_gripper()
self.send_move(pregrasp)
self.send_move(grasp)
self.close_gripper()
self.stop_gripper()
self.send_move(lift)
self.send_move(home)
```

Where `grasp_offset` must be tuned experimentally according to the block height and gripper geometry.

Start with a conservative value and move slowly.

## 9. Run your code

You need two terminals.

Terminal 1: launch the pose estimation stack.

```bash
cd ~/dev_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch pose_estimation_cobot rviz_pose_estimation.launch.py robot_ip:=<robot_ip>
```

Terminal 2: run your pick node.

```bash
cd ~/dev_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pick_block_pkg pick_block
```

## 10. Safety checklist

Before running the pick sequence on the real robot, verify:

* The emergency stop is accessible.
* The robot is moving at low speed.
* The table is clear.
* The centroid is stable in RViz.
* The `pregrasp` position is above the block, not inside the table.
* The gripper is open before moving down.
* The robot can reach the position without singularities or collisions.

> If you are not sure where the TCP will move, do not execute the motion.

## 11. Challenge

### Challenge 1

Print the centroid position and move the robot only to the `pregrasp` position.

### Challenge 2

Complete the full pick sequence.

### Challenge 3

Add a second position and place the block in a new location.

### Challenge 4

Add workspace limits so the robot ignores detections outside the safe area.

