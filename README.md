# 🤖 HM2 Project: IIWA VISUAL CONTROL with ARUCO_ROS

This repository contains the solution for **Homework 2** of the Robotics Lab class, focused on building ROS packages to simulate a vision-based controller for a robotic manipulator arm in the simulation environment using KDL and aruco_ros.

## 📂 Repository Structure

HM2
├README
│
├ aruco_ros/  
├ ros2_iiwa/      
└ ros2_kdl_package/  

     	
     	
# 🦾 HM2 Setup & Usage Guide


---

## 🚀 0. Install All Necessary Libraries

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros2-control \
  ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-kdl-parser \
  liborocos-kdl-dev \
  libopencv-dev \
  libeigen3-dev
---
```
## 🤖 1a. iiwa parameters

The following variables become ROS2 params: traj_duration, acc_duration, total_time, trajectory_len, Kp, and the three components of the trajectory end_position. We create a launch file that starts the ros2_kdl_node loading a .yaml file (from a config folder) that contains the aforementioned parameters’ definition

### 🧩 Launch Command

```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:=position robot_controller:=iiwa_arm_controller
ros2 launch ros2_kdl_package ros2_kdl_node_launch.py
```

---

## 🌍 1b. Control null

We create a new controller in the kdl_control class called velocity_ctrl_null that implements a velocity control law with the contribution of the null space. 

### 🚀 Launch Simulation

```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:=velocity robot_controller:=velocity_controller
```

### 🎮 Send a Control Command

We can also switch between the two velocity controllers creating an additional parameter ctrl:=velocity_ctrl|velocity_ctrl_null passed as argument to the node.

#### Example Command

```bash
ros2 launch ros2_kdl_package ros2_kdl_node_launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null 
```
```bash
ros2 launch ros2_kdl_package ros2_kdl_node_launch.py cmd_interface:=velocity ctrl:=velocity_ctrl
```

---

## 🧠 1c. action server/client

We make our ros2_kdl_node an action server that executes the same linear trajectory and publishes the position error as feedback and also an action client is written to test our code.

### ▶️ Run the Controller Node

```bash
ros2 launch iiwa_bringup iiwa.launch.py command_interface:=position robot_controller:=iiwa_arm_controller
```

```bash
ros2 launch ros2_kdl_package ros2_kdl_node_launch.py enable_action_server:=true
```

```bash
ros2 run ros2_kdl_package trajectory_action_client
```

---

## ⚙️ 2a. 

We construct a gazebo world by inserting an aruco tag and detecting it via the aruco_ros package. We create a folder gazebo/models that contains the aruco marker model for gazebo. We use an aruco tag model that we imported into the new gazebo world.

### 🧩 How to run

```bash
ros2 launch iiwa_bringup iiwa.launch.py   use_sim:=true start_rviz:=false   gz_args:="-r -v 1 $(ros2 pkg prefix iiwa_description)/share/iiwa_description/gazebo/worlds/iiwa_aruco.world"
```


## 🔧 2b. 

The controller subscribes to the ArUco marker pose topic and computes the joint velocity commands according to the following control law:

$\dot{q} = K(L(s)J_c)^{\dagger} s_d + N\dot{q}_0$

where:

-K is a diagonal gain matrix

-sd = [0, 0, 1] is the desired direction

-Jc is the camera Jacobian

-L(s) maps the camera linear/angular velocities to changes in s

The implemented controller aligns the camera optical axis with the direction of the ArUco marker, ensuring the robot always "looks at" the marker.

### 📎 How to run

```bash
ros2 launch iiwa_bringup iiwa.launch.py   use_sim:=true   command_interface:=velocity   robot_controller:=velocity_controller   start_rviz:=false   gz_args:="-r -v 1 $(ros2 pkg prefix iiwa_description)/share/iiwa_description/gazebo/worlds/iiwa_aruco.world"
```

```bash
ros2 launch ros2_kdl_package camera_bridge.launch.py
```

```bash
ros2 run aruco_ros single --ros-args \
  -p marker_id:=201 \
  -p marker_size:=0.1 \
  -p camera_frame:=link_7_optical_frame \
  -p marker_frame:=aruco_marker_frame \
  -p reference_frame:=link_7_optical_frame \
  -r /image:=/wrist_camera/image_raw \
  -r /camera_info:=/wrist_camera/camera_info
```

```bash
ros2 launch ros2_kdl_package ros2_kdl_node_launch.py   cmd_interface:=velocity   ctrl:=vision
```


To verify that the camera recognizes the marker:

```bash
ros2 run rqt_image_view rqt_image_view image:=/aruco_single/result
```

---



## 📡 2c. 

In this point we created a service that updates the marker position in Gazebo. We modified the ros2_kdl_node.launch.py file where we implemented the parameter_bridge that auto-detects messages for setting the ArUco pose in Gazebo, so that a user can test the code with a ros2 service call command, where in the position and orientation brackets the user can set the  preferred values.

### 📏 How to run

```bash
ros2 launch iiwa_bringup iiwa.launch.py   use_sim:=true   command_interface:=velocity   robot_controller:=velocity_controller   start_rviz:=false   gz_args:="-r -v 1 $(ros2 pkg prefix iiwa_description)/share/iiwa_description/gazebo/worlds/iiwa_aruco.world"
```

```bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py cmd_interface:=velocity   ctrl:=vision
```

```bash
ros2 service call /world/iiwa_aruco_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'aruco_tag_front'}, pose: {position: {x: 0.1, y: -0.5, z: 0.6}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

