# Line Following Robot Using ROS 2 and Gazebo

**Student:** Chaima Saoudi  
**Year:** 2024-2025  
**Technologies:** ROS 2 Humble, Gazebo 11, Python 3, OpenCV

---

## Project Structure

This project contains three main directories:

### 1. turtlebot3_ws - Main ROS 2 Workspace
Contains the line_follower package with detection and control code.

### 2. my_line_follower_world - Gazebo Worlds
Contains 8 different test environments:
- line_world.world - Simple straight line
- advanced_line_world.world - Circuit with curves
- zigzag_world.world - Zigzag trajectory
- zigzag_continuous.world - Smooth zigzag
- zigzag_with_obstacles.world - Zigzag with obstacles
- city_road_world.world - Urban realistic environment
- smooth_curve_world.world - Smooth curves
- racing_circuit_world.world - Racing track

### 3. ros2_ws - Additional Simulation Workspace

---

## Quick Start

**1. Build the workspace:**
```bash
cd turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

**2. Launch Urban Environment (Terminal 1):**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_city_road.launch.py
```

**3. Run Line Follower (Terminal 2):**
```bash
source turtlebot3_ws/install/setup.bash
ros2 run line_follower line_follower_node
```

---

## Features

✅ Vision-based line detection using OpenCV  
✅ HSV color space for white/yellow lines  
✅ Proportional control at 0.8 m/s  
✅ 3-phase line loss recovery system  
✅ 8 different test environments  
✅ Real-time performance (30+ FPS)  

---

## Performance Results

- **Tracking Precision:** ±5 pixels
- **Maximum Speed:** 0.8 m/s
- **Success Rate:** 95%
- **Recovery Rate:** 90%

---

## Contact

**Chaima Saoudi**  
Email: chaimasaoudi186@gmail.com  
GitHub: @ChaimaSaoudi23
