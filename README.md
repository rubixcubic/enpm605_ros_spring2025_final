# enpm605_ros_spring2025_final
Package for the final project

## Prerequisites

- Clone this repository in the new workspace.
- Build the simulation environment.

```bash
colcon build --symlink-install --packages-up-to rosbot --cmake-args -DCMAKE_BUILD_TYPE=Release
```
- Build the Python demo packages:
```bash
colcon build --symlink-install --packages-select ros2_aruco_interfaces ros2_aruco
```

## Testing

Start the environment:

```bash
ros2 launch rosbot_gazebo simulation.launch.py
```

Check the Gazebo camera is reporting the detected AruCo marker:


```bash
ros2 topic echo /aruco_markers
```


