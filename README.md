# ROS Particle Filter Localization

A Monte Carlo Localization (MCL) implementation using particle filters for underwater robot localization in ROS with Gazebo simulation.

## Overview

This project implements a particle filter-based localization system for the AQUA underwater robot using ROS. The system includes ground truth tracking from Gazebo, pose estimation visualization, and trajectory comparison between estimated and actual robot positions.

## Features

- Monte Carlo Localization (MCL) algorithm implementation
- Ground truth pose tracking from Gazebo simulator
- Real-time trajectory visualization and comparison
- Error metric computation between estimated and ground truth poses
- RViz configuration for 3D trajectory visualization
- Image-based map overlay with robot pose visualization

## Project Structure

```
ROS-Particle-Filter/
├── particle_filter/              # Main ROS package
│   ├── src/
│   │   ├── ground_truth_publisher.cpp  # Publishes ground truth and visualizes trajectories
│   │   └── localizer_node.cpp          # Particle filter localization node
│   ├── rviz/
│   │   └── config.rviz                 # RViz visualization configuration
│   ├── CMakeLists.txt
│   └── package.xml
├── .vscode/
│   └── settings.json
├── .catkin_workspace
├── .gitignore
└── README.md
```

## Dependencies

### ROS Packages
- `roscpp` - ROS C++ client library
- `cv_bridge` - OpenCV-ROS image conversion
- `image_transport` - Image publishing/subscribing
- `sensor_msgs` - Standard sensor message definitions
- `geometry_msgs` - Geometric message definitions
- `tf` - Transform library
- `gazebo_msgs` - Gazebo simulator messages

### External Libraries
- OpenCV 2+ - Computer vision and image processing
- Gazebo - Robot simulation environment
- AQUA Gazebo package - AQUA robot simulation files

## Installation

1. **Clone the repository into your catkin workspace:**
   ```bash
   cd ~/catkin_ws/src
   git clone <repository-url> ROS-Particle-Filter
   ```

2. **Install dependencies:**
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Running the Localization System

1. **Start the Gazebo simulation with AQUA robot:**
   ```bash
   roslaunch aqua_gazebo <your_launch_file>.launch
   ```

2. **Launch the ground truth publisher:**
   ```bash
   rosrun particle_filter ground_truth_publisher
   ```

3. **Run the localizer node:**
   ```bash
   rosrun particle_filter localizer_node
   ```

4. **Open RViz for visualization:**
   ```bash
   rviz -d src/ROS-Particle-Filter/particle_filter/rviz/config.rviz
   ```

## ROS Topics

### Subscribed Topics
- `/gazebo/model_states` - Ground truth robot pose from Gazebo
- `/assign1/localization_estimate` - Estimated pose from particle filter

### Published Topics
- `/assign1/result_image` - Visualization image with trajectories
- `/assign1/gt_trajectory` - Ground truth trajectory as PoseArray
- `/assign1/estimate_trajectory` - Estimated trajectory as PoseArray

## Visualization

The system provides multiple visualization methods:

1. **RViz Trajectories**: 
   - Blue arrows: Ground truth trajectory
   - Red arrows: Estimated trajectory

2. **Image Overlay**:
   - Map image with robot positions overlaid
   - Blue circles/lines: Ground truth pose and heading
   - Red circles/lines: Estimated pose and heading

## Error Metrics

The ground truth publisher computes and displays:
- Current position error (Euclidean distance)
- Cumulative total error
- Position and orientation comparison

Error is printed to console in real-time:
```
ESTIMATE: [x, y, yaw]=[...]
GR TRUTH: [x, y, yaw]=[...]
CUR ERROR: ...
TOT ERROR: ...
```

## Configuration

### Parameters
- `METRE_TO_PIXEL_SCALE`: 50 - Conversion ratio for map visualization
- `POSITION_GRAPHIC_RADIUS`: 20.0 pixels - Size of position markers
- `HEADING_GRAPHIC_LENGTH`: 50.0 pixels - Length of heading arrows

### RViz Configuration
The included `config.rviz` file provides:
- Orbit camera view
- Grid reference frame
- PoseArray displays for both trajectories
- Color-coded trajectory visualization

## License

MIT License

## Maintainer

Nikhil Jaiyam (nikhiljaiyam6@gmail.com)
