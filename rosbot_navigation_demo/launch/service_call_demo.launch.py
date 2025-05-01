from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory("rosbot_navigation_demo")

    # Define paths
    goals_path = os.path.join(package_dir, "config", "goals.yaml")

    # Launch arguments
    map_frame_arg = DeclareLaunchArgument(
        "map_frame", default_value="odom", description="Frame ID for the map"
    )
    rotation_speed_arg = DeclareLaunchArgument(
        "rotation_speed",
        default_value="0.3",
        description="Robot rotation speed when searching for color",
    )
    service_call_type_arg = DeclareLaunchArgument(
        "service_call_type", default_value="async", description="Type of service call (async or sync)"
    )
    target_color_arg = DeclareLaunchArgument(
        "target_color",
        default_value="red",
        description="Target color for navigation (red, blue, green, or yellow)",
    )
    min_detection_area_arg = DeclareLaunchArgument(
        "min_detection_area",
        default_value="30",
        description="Minimum area for color detection in pixels",
    )

    # Nodes
    color_beacon_node = Node(
        package="rosbot_navigation_demo",
        executable="color_beacon_demo",
        output="screen",
        parameters=[
            {
                "target_color": LaunchConfiguration("target_color"),
                # Red color parameters (special case with two ranges)
                "red_lower_hsv": [0, 30, 30],
                "red_upper_hsv": [15, 255, 255],
                "red_lower_hsv2": [160, 30, 30],
                "red_upper_hsv2": [179, 255, 255],
                # Blue color parameters
                "blue_lower_hsv": [100, 30, 30],
                "blue_upper_hsv": [130, 255, 255],
                # Green color parameters
                "green_lower_hsv": [40, 30, 30],
                "green_upper_hsv": [80, 255, 255],
                # Yellow color parameters
                "yellow_lower_hsv": [20, 30, 30],
                "yellow_upper_hsv": [35, 255, 255],
                # Other parameters
                "min_detection_area": LaunchConfiguration("min_detection_area"),
                "rotation_speed": LaunchConfiguration("rotation_speed"),
                "rotation_timer_period": 0.1,
            }
        ],
    )

    get_goal_service_node = Node(
        package="rosbot_navigation_demo",
        executable="get_goal_service_demo",
        output="screen",
        parameters=[
            {"goals_file": goals_path, "map_frame": LaunchConfiguration("map_frame")}
        ],
    )
    
    cube_publisher_node = Node(
        package="rosbot_navigation_demo",
        executable="cube_publisher_demo",
        output="screen",
    )

    navigation_manager_node = Node(
        package="rosbot_navigation_demo",
        executable="navigation_manager_demo",
        output="screen",
        parameters=[
            {
                "target_color": LaunchConfiguration("target_color"),
                "cooldown_seconds": 5.0,
                "distance_tolerance": 0.1,
                "angle_tolerance": 0.05,
                "navigation_timeout": 60.0,
                "min_confidence": 0.0,  # Lower to detect any color presence
                "expected_frame_id": LaunchConfiguration("map_frame"),
                "service_call_type": LaunchConfiguration("service_call_type"),
            }
        ],
    )
    
    move_to_goal_action_node = Node(
        package="rosbot_navigation_demo",
        executable="move_to_goal_action_demo",
        output="screen",
        parameters=[
            {
                "kp_linear": 0.5,
                "kp_angular": 1.0,
                "default_linear_tolerance": 0.1,
                "default_angular_tolerance": 0.2,
                "control_frequency": 20.0,
                "goal_timeout": 60.0,
            }
        ],
    )


    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(map_frame_arg)
    ld.add_action(rotation_speed_arg)
    ld.add_action(target_color_arg)
    ld.add_action(min_detection_area_arg)
    ld.add_action(service_call_type_arg)

    # Add nodes
    # ld.add_action(cube_publisher_node)
    ld.add_action(color_beacon_node)
    ld.add_action(get_goal_service_node)
    ld.add_action(move_to_goal_action_node)
    ld.add_action(navigation_manager_node)

    return ld
