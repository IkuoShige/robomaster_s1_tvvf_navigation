# RoboMaster S1 TVVF Navigation

TVVF-based navigation and waypoint following system for RoboMaster S1 robot with Livox MID-360 LiDAR.

## Features

- **Integrated Navigation Stack**: Complete localization, planning, and waypoint following
- **No Nav2 Dependency**: Lightweight `simple_map_server` implementation
- **Unified Launch**: Single launch file for both simulation and real robot
- **Mecanum Wheel Support**: Works with RoboMaster S1's mecanum wheel drive
- **CSV Waypoint Files**: Compatible with [waypoint_editor](https://github.com/kzm784/waypoint_editor.git)
- **Advanced Waypoint Commands**: Supports wait, pause, topic-based resume

## Package Contents

### Navigation Stack
- **map_scan_manager**: 3D LiDAR to 2D laser scan conversion
- **simple_map_server**: Lightweight map server (Nav2-free)
- **emcl2**: Monte Carlo Localization
- **obstacle_tracker**: DBSCAN-based obstacle detection
- **tvvf_vo_c**: Time-Varying Vector Field local planner
- **waypoint_follower**: CSV-based waypoint navigation

### Directory Structure
```
robomaster_s1_tvvf_navigation/
├── config/                  # All configuration files
│   ├── emcl2_params.yaml
│   ├── map_scan_params.yaml
│   ├── map_regions.yaml
│   ├── obstacle_tracker_params.yaml
│   ├── tvvf_vo_params.yaml
│   └── waypoint_follower_params.yaml
├── include/
│   └── robomaster_s1_tvvf_navigation/
│       ├── csv_reader.hpp
│       ├── simple_map_server.hpp
│       ├── waypoint_follower_node.hpp
│       └── waypoint_manager.hpp
├── launch/
│   └── waypoint_navigation.launch.py
├── maps/                    # Map files (PGM + YAML)
├── msg/
│   └── WaypointStatus.msg
├── rviz/
│   └── navigation.rviz
└── src/                     # Implementation files
```

## Dependencies

### Navigation Stack
- `emcl2`: Monte Carlo Localization
- `map_scan_manager`: LiDAR scan management
- `obstacle_tracker`: Obstacle detection
- `tvvf_vo_c`: Local planner (feat/waypoint_navigation_s1 branch)

### Build Dependencies
- `yaml-cpp`: YAML parsing for map server
- `OpenCV`: Image loading for map server

### ROS 2 Packages
- `rclcpp`
- `geometry_msgs`
- `nav_msgs`
- `visualization_msgs`
- `std_msgs`
- `std_srvs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `sensor_msgs`

## Installation

```bash
cd ~/s1_ws/src/ros2_ws
colcon build --packages-select robomaster_s1_tvvf_navigation --symlink-install
source install/setup.bash
```

## Usage

### Gazebo Simulation

```bash
ros2 launch robomaster_s1_gazebo s1_navigation_gazebo.launch.py
```

### Real Robot

```bash
ros2 launch robomaster_s1_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=false
```

### Launch Arguments

- `use_sim_time`: Use simulation clock (default: `false`)
- `map_file`: Path to map YAML file (default: `maps/maps.yaml`)
- `rviz`: Launch RViz2 (default: `true`)

## Waypoint CSV Format

CSV files should follow this format (compatible with [waypoint_editor](https://github.com/kzm784/waypoint_editor.git)):

```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command
0,1.0,2.0,0.0,0.0,0.0,0.0,1.0,
1,3.0,4.0,0.0,0.0,0.0,0.707,0.707,wait:5
2,5.0,6.0,0.0,0.0,0.0,0.0,1.0,pause
3,7.0,8.0,0.0,0.0,0.0,1.0,0.0,wait_topic:/resume_signal
```

### Supported Commands

- **Empty**: Continue to next waypoint
- **wait:N**: Wait N seconds before continuing
- **pause**: Stop and wait for manual resume
- **wait_topic:/topic_name**: Wait for Bool message on topic

## Services

- `/start_waypoint_navigation` (std_srvs/Trigger): Start waypoint navigation
- `/stop_waypoint_navigation` (std_srvs/Trigger): Stop waypoint navigation
- `/resume_waypoint_navigation` (std_srvs/Trigger): Resume from pause
- `/skip_waypoint` (std_srvs/Trigger): Skip current waypoint

## Topics

### Published
- `/waypoint_path` (nav_msgs/Path): Complete waypoint path
- `/current_waypoint_marker` (visualization_msgs/Marker): Current target waypoint
- `/waypoint_status` (robomaster_s1_tvvf_navigation/WaypointStatus): Navigation status

### Subscribed
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): Robot pose from localization

## Configuration

All configuration is centralized in `config/` directory. No launch file arguments needed for most use cases.

### Key Parameters

**emcl2** (localization):
- Particle count, resampling thresholds
- Expansion resetting parameters

**map_scan_manager**:
- Height-based scan filtering
- Multi-map region support

**obstacle_tracker**:
- DBSCAN clustering parameters
- Dynamic obstacle detection

**tvvf_vo_c** (local planner):
- Mecanum wheel kinematics
- Vector field parameters
- Obstacle avoidance settings

**waypoint_follower**:
- Waypoint reach threshold
- Auto-start behavior
- CSV file path

## Migration from Old Packages

This package replaces:
- `robomaster_s1_navigation`
- `robomaster_s1_waypoint_navigation`

See `DEPRECATED.md` in those packages for migration instructions.

## Architecture

This package follows the same unified architecture as `raspicat_tvvf_navigation`:
1. Single integrated package
2. Nav2-free implementation
3. Centralized configuration
4. Simulation/real robot toggle via `use_sim_time`

## License

Apache-2.0

## References

- Waypoint Editor: https://github.com/kzm784/waypoint_editor.git
- TVVF Local Planner: tvvf_vo_cpp (feat/waypoint_navigation_s1 branch)
