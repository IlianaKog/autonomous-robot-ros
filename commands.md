# ROS 2 Autonomous Outdoor Robot - Launch Commands

## 1. First-Time Setup & Full Build

If this is your first time cloning the project, or if you modified C++ files, you need to do a full build of the workspace. Open a WSL (Ubuntu) terminal and run:

```bash
cd /mnt/c/Users/User/Desktop/autonomous-robot-ros
colcon build --symlink-install
```

*(Optional)* If you only modified the mission node C++ file, you can do a faster, targeted build:
```bash
cd /mnt/c/Users/User/Desktop/autonomous-robot-ros
colcon build --packages-select outdoor_robot_mission
```

---

## 2. Launching the Simulation

To start the robot and execute the inspection mission, open 4 separate WSL terminals and run the following commands in order:

### Terminal 1: Launch World & Robot
```bash
cd /mnt/c/Users/User/Desktop/autonomous-robot-ros
source install/setup.bash
ros2 launch outdoor_robot_bringup bringup.launch.py
```
*(This terminal will print a few lines and then remain quiet.)*

### Terminal 2: Launch Nav2 (Path Planner)
```bash
cd /mnt/c/Users/User/Desktop/autonomous-robot-ros
source install/setup.bash
ros2 launch outdoor_robot_navigation navigation.launch.py
```
*(Wait until it outputs "Creating bond timer..." before proceeding.)*

### Terminal 3: Launch Camera Node (Target Detection)
```bash
cd /mnt/c/Users/User/Desktop/autonomous-robot-ros
source install/setup.bash
ros2 run outdoor_robot_perception inspection_behavior_node
```
*(It will output: "Perception node started. Waiting for red targets...")*

### Terminal 4: Launch Mission (C++ Waypoints)
```bash
cd /mnt/c/Users/User/Desktop/autonomous-robot-ros
source install/setup.bash
ros2 run outdoor_robot_mission gps_waypoint_follower
```
*(Here the robot will receive the (X,Y) goals and start moving!)*
