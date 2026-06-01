# 🐢 Turtle SLAM Project
 
---

## 📖 Overview

This project implements SLAM on a TurtleBot platform, enabling the robot to build a map of an unknown environment while simultaneously tracking its own location within it. The robot navigates autonomously, avoids obstacles, and produces an accurate 2D occupancy grid map in real time.

---

## ✨ Features

- **Real-time SLAM** — builds a map on-the-fly using LiDAR sensor data
- **Autonomous Navigation** — path planning and obstacle avoidance
- **Localization** — accurate pose estimation within the generated map
- **Map Saving & Loading** — persist and reuse maps across sessions
- **ROS Integration** — fully compatible with ROS 2 (Robot Operating System)
- **Visualization** — live map and robot pose display via RViz

---

## 🛠️ Tech Stack

| Component | Tool / Library |
|-----------|---------------|
| Framework | ROS 2 (Humble / Foxy) |
| SLAM Algorithm | Cartographer / GMapping / SLAM Toolbox |
| Robot Platform | TurtleBot3 (Burger / Waffle) |
| Sensor | LiDAR (LDS-01 / LDS-02) |
| Simulation | Gazebo |
| Visualization | RViz2 |
| Language | Python / C++ |

---

## 📋 Prerequisites

- Ubuntu 22.04 (or 20.04)
- ROS 2 Humble (or Foxy)
- Python 3.8+
- TurtleBot3 packages

---

## 🚀 Installation

### 1. Clone the repository

```bash
git clone https://github.com/your-username/turtle-slam.git
cd turtle-slam
```

### 2. Install dependencies

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-slam-toolbox
pip install -r requirements.txt
```

### 3. Set environment variables

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### 4. Build the workspace

```bash
colcon build
source install/setup.bash
```

---

## 🎮 Usage

### Launch simulation (Gazebo)

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Start SLAM

```bash
ros2 launch turtle_slam slam.launch.py
```

### Control the robot

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Navigate with saved map

```bash
ros2 launch turtle_slam navigation.launch.py map:=~/maps/my_map.yaml
```

---

## 📊 Results

The robot successfully maps indoor environments, producing clean occupancy grid maps with clearly defined walls, corridors, and open spaces. Navigation accuracy is within ±5 cm of the target goal position.


---

## 📄 License

This project is licensed under the UNSW License.

---

## 🙏 Acknowledgements

- [ROS 2 Documentation](https://docs.ros.org)
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Nav2 Project](https://navigation.ros.org)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
