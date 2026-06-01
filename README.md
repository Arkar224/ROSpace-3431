# Comp3431 (Robotic Software Architecture) ROSpace

This repository is created for team ROSpace to control a physical TurtleBot3 Waffle robot using ROS2 architecture. The project implements wall following, landmark detection, and autonomous navigation using colored markers.

## Project Overview

This is a ROS2-based robotics project for the TurtleBot3 Waffle that includes:
- **Wall Following**: Navigate through maze environments by following walls
- **Landmark Detection**: Identify and track colored cylindrical markers using computer vision
- **Autonomous Navigation**: Navigate to waypoints using Nav2 stack
- **Color Calibration**: Tool for calibrating HSV color ranges for marker detection
- **Mapping**: SLAM-based mapping using cartographer

## Directory Structure

```
ROSpace-3431/
├── README.md                    # This file - project documentation
├── landmarks.csv                # Detected landmark positions (x, y, marker_type)
├── map/                         # Map files for navigation
│   ├── map.pgm                  # Occupancy grid map image
│   └── map.yaml                 # Map metadata (resolution, origin, thresholds)
└── src/                         # Source code directory
    ├── color_retriever.py       # Interactive color calibration tool
    └── wall_follower/           # Main ROS2 package
        ├── CMakeLists.txt       # CMake build configuration
        ├── package.xml          # ROS2 package metadata
        ├── README               # Launch instructions for the package
        ├── startup.bash         # Automated startup script
        ├── landmarks.csv        # Package-level landmarks file
        ├── waypoints.csv        # Navigation waypoints (x, y coordinates)
        ├── config/              # Configuration files
        │   └── waypoint_nav_params.yaml  # Nav2 navigation parameters
        ├── launch/              # Launch files
        │   ├── wall_follower.launch.py   # Launch wall follower system
        │   └── waypoint_navigator.launch.py  # Launch navigation system
        ├── scripts/             # Python executable scripts
        │   ├── see_marker.py            # Vision-based marker detection
        │   ├── point_transformer.py     # Transform marker positions to map frame
        │   └── waypoint_navigator.py    # Autonomous waypoint navigation
        ├── src/                 # C++ source files
        │   └── wall_follower.cpp        # Wall following algorithm
        ├── wall_follower/       # Python module
        │   ├── __init__.py              # Package initialization
        │   └── landmark.py              # Landmark data structures
        ├── include/             # C++ header files
        ├── model_editor_models/ # Gazebo simulation models
        └── screenshots/         # Documentation images
```

## Key Files and Their Functions

### Root Level Files

- **`landmarks.csv`**: Output file containing detected landmark positions in format: `x,y,marker_type`
- **`map/map.pgm`**: Occupancy grid map generated from SLAM (Simultaneous Localization and Mapping)
- **`map/map.yaml`**: Map metadata including resolution (0.05m/pixel), origin coordinates, and occupancy thresholds

### Source Files

#### Color Calibration Tool
- **`src/color_retriever.py`**: Interactive ROS2 node for HSV color range calibration
  - Subscribes to `/camera/image_raw` topic
  - Provides GUI for selecting color samples by clicking on pixels
  - Supports calibration for 4 colors: Green (G), Blue (B), Yellow (Y), Pink (P)
  - Outputs HSV color bounds for use in marker detection
  - **Controls**:
    - Click pixels to sample colors in current mode
    - `G/B/Y/P` keys: Switch between color modes
    - `C` key: Clear current color samples
    - `O` key: Output color bounds to console
    - `ESC` key: Exit application

#### Wall Follower Package (`src/wall_follower/`)

##### C++ Components
- **`src/wall_follower.cpp`**: Main wall following algorithm
  - Subscribes to `/scan` (LaserScan) for distance sensing
  - Subscribes to `/odom` (Odometry) for robot position
  - Publishes to `/cmd_vel` (Twist) for robot motion control
  - Implements wall following behavior using laser scan data
  - Uses PID-style control to maintain constant distance from walls

##### Python Scripts
- **`scripts/see_marker.py`**: Computer vision node for marker detection
  - Subscribes to `/camera/image_raw` for camera feed
  - Subscribes to `/scan` (LaserScan) for distance measurements
  - Uses HSV color segmentation to detect colored markers
  - Applies bilateral filtering for noise reduction
  - Detects two-color cylindrical landmarks (e.g., yellow/pink, blue/pink)
  - Publishes detected marker positions as `PointStamped` messages to `/marker_position`
  - Supports 6 marker types: yellow/pink, green/pink, blue/pink, pink/yellow, pink/green, pink/blue

- **`scripts/point_transformer.py`**: Coordinate transformation node
  - Subscribes to `/marker_position` (PointStamped) from see_marker.py
  - Uses TF2 to transform marker positions from camera frame to map frame
  - Maintains running average of marker positions for stability
  - Publishes `MarkerArray` visualization messages for RViz
  - Saves landmark positions to `landmarks.csv` on shutdown

- **`scripts/waypoint_navigator.py`**: Autonomous navigation node
  - Uses Nav2 (Navigation2) stack for path planning and execution
  - Reads waypoint coordinates from CSV file (configurable parameter)
  - Sequentially navigates through all waypoints
  - Monitors navigation task completion and handles failures
  - Default waypoints file: `/home/troublemaker/comp3431/turtlebot_ws/landmarks.csv`

##### Python Module
- **`wall_follower/landmark.py`**: Landmark data structures and utilities
  - Defines `Landmark` class for storing marker information
  - Maintains position averaging for stable marker localization
  - Creates RViz visualization markers (cylinders with two colors)
  - Defines 6 marker types with color combinations
  - Exports landmark data to CSV format

##### Configuration
- **`config/waypoint_nav_params.yaml`**: Nav2 navigation parameters
  - Controller, planner, and behavior server configurations
  - Robot footprint and collision checking parameters
  - Path planning algorithms and tolerances
  - Recovery behavior settings

##### Launch Files
- **`launch/wall_follower.launch.py`**: Launches the wall following system
  - Starts three nodes simultaneously:
    1. `wall_follower` (C++ node) - wall following control
    2. `see_marker.py` - marker detection
    3. `point_transformer.py` - coordinate transformation

- **`launch/waypoint_navigator.launch.py`**: Launches autonomous navigation
  - Includes Nav2 navigation stack
  - Starts waypoint navigator node
  - Configurable parameters:
    - `params_file`: Navigation parameters YAML
    - `map`: Map file path
    - `waypoints_file`: CSV file with waypoints
    - `use_sim_time`: True for simulation, False for real robot

##### Data Files
- **`landmarks.csv`**: Package-level landmark storage
- **`waypoints.csv`**: List of waypoint coordinates for navigation

## Dependencies

This project requires:
- **ROS2** (tested on ROS2 Humble or later)
- **TurtleBot3 packages**:
  - `turtlebot3_gazebo` - Simulation environment
  - `turtlebot3_cartographer` - SLAM functionality
  - `turtlebot3_navigation2` - Navigation stack
- **Python packages**:
  - `rclpy` - ROS2 Python client library
  - `opencv-python` (cv2) - Computer vision
  - `numpy` - Numerical operations
  - `cv_bridge` - ROS-OpenCV conversion
  - `nav2_simple_commander` - Navigation interface
- **C++ libraries**:
  - `rclcpp` - ROS2 C++ client library
  - `geometry_msgs`, `nav_msgs`, `sensor_msgs` - ROS2 message types
  - `tf2` - Transform library

## Building the Project

```bash
# Navigate to workspace root
cd /path/to/your/ros2_workspace

# Build the wall_follower package
colcon build --packages-select wall_follower

# Source the workspace
source install/setup.bash
```

## Usage

### Method 1: Automated Startup (Simulation)

Use the provided startup script to launch all required components:

```bash
cd src/wall_follower
bash startup.bash
```

This script will:
1. Launch TurtleBot3 in Gazebo maze environment
2. Start cartographer for SLAM (mapping)
3. Launch the wall follower system

### Method 2: Manual Launch (Recommended for Real Robot)

Launch each component in a separate terminal:

**Terminal 1 - Simulation Environment** (skip for real robot):
```bash
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py
```

**Terminal 2 - SLAM/Mapping**:
```bash
# For simulation:
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# For real robot:
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=False
```

**Terminal 3 - Wall Follower**:
```bash
ros2 launch wall_follower wall_follower.launch.py
```

**Optional - Autonomous Navigation**:
```bash
ros2 launch wall_follower waypoint_navigator.launch.py \
    map:=/path/to/your/map.yaml \
    waypoints_file:=/path/to/waypoints.csv \
    use_sim_time:=true  # or false for real robot
```

### Color Calibration

To calibrate colors for marker detection:

```bash
# Ensure robot/simulation is running with camera
python3 src/color_retriever.py
```

Follow the on-screen instructions to select color samples and export HSV bounds.

## Visualization

To visualize the robot, map, and detected markers:

```bash
rviz2
```

In RViz:
1. Set Fixed Frame to `map`
2. Add displays for:
   - RobotModel
   - Map
   - LaserScan
   - MarkerArray (for visualizing detected landmarks)
   - Path (for navigation trajectories)

## CSV File Formats

### landmarks.csv
```
x_coordinate,y_coordinate,marker_type_id
-0.045,-3.168,0
-0.071,-1.037,2
```
- `x_coordinate`: X position in map frame (meters)
- `y_coordinate`: Y position in map frame (meters)
- `marker_type_id`: Integer 0-5 representing marker type

### waypoints.csv
```
x_coordinate,y_coordinate
0.0,0.0
1.5,2.0
```
- Each line contains a waypoint coordinate in map frame

## Marker Types

The system recognizes 6 two-color marker types:
0. Yellow/Pink (yellow top, pink bottom)
1. Green/Pink
2. Blue/Pink
3. Pink/Yellow (pink top, yellow bottom)
4. Pink/Green
5. Pink/Blue

## Troubleshooting

- **No camera image**: Check camera topic name in scripts (default: `/camera/image_raw`)
- **Markers not detected**: Recalibrate colors using `color_retriever.py`
- **Navigation fails**: Ensure map is loaded and robot is localized
- **TF errors**: Check that all transforms are being published correctly

## License

Released under GPLv3. Most resources are written by UNSW demonstrators and students.

## Authors

- Claude Sammut (Main contributor)
- UNSW COMP3431 Teaching Team
- Team ROSpace
