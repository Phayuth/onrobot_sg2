# Onrobot Soft Gripper ROS Package
ROS2 package for Onrobot Soft Gripper. Featuring gripper :

- SG-a
- SG-b

Rviz visualization only contain static model, since soft gripper is difficult to simulated and animate.

## Dependencies and Installation
```
sudo pip3 install -U pymodbus
```
Clone and build the pacakge as usual with `colcon build`

## Usage
- Bring up the gripper. Enter the launch folder
```
ros2 launch bringup_gripper.launch.py
```

- Control the gripper via rosservice, where desired width is between 110mm and 750 mm
```
ros2 service call /gripper_command onrobotsg_interfaces/srv/Sg desiredwidth:\ desired width\
```

- Control via client service
```
ros2 run onrobotsg onrobotsg_client 75
```