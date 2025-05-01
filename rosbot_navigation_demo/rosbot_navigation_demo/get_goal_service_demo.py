import rclpy
from rclpy.node import Node
from rosbot_interfaces.srv import GetGoal
from geometry_msgs.msg import PoseStamped
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import time

class GoalProviderService(Node):
    def __init__(self):
        super().__init__("get_goal_service_demo")

        # Parameters
        self.declare_parameter("goals_file", "goals.yaml")
        self.declare_parameter("map_frame", "odom")

        self._goals_file = self.get_parameter("goals_file").value
        self._map_frame = self.get_parameter("map_frame").value

        # Load predefined goals
        self._goals = self._load_goals()

        # Create GetGoal service
        self._get_goal_srv = self.create_service(
            GetGoal, "/get_goal", self._handle_get_goal_request
        )

        self.get_logger().info("get_goal_service_demo node initialized")
        self.get_logger().info(f"Loaded {len(self._goals)} predefined goals")

    def _load_goals(self):
        """Load goal positions from YAML file"""
        goals = {}

        # Try to find the goals file in various locations
        goals_path = self.get_parameter("goals_file").value

        # If not an absolute path, check in the package share directory
        if not os.path.isabs(goals_path):
            try:
                package_dir = get_package_share_directory("rosbot_navigation_demo")
                goals_path = os.path.join(package_dir, "config", goals_path)
            except Exception as e:
                self.get_logger().warn(f"Failed to locate package directory: {e}")

        # If file doesn't exist, use default values
        if not os.path.exists(goals_path):
            self.get_logger().warn(
                f"Goals file not found at {goals_path}, using default values"
            )
            # Default goals if file not found
            goals = {
                "0": {
                    "position": {"x": 1.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "1": {
                    "position": {"x": 0.0, "y": 1.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "2": {
                    "position": {"x": -1.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "3": {
                    "position": {"x": 4.0, "y": -2.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            }
            return goals

        # Try to load the file
        try:
            with open(goals_path, "r") as file:
                goals = yaml.safe_load(file)
                if not goals:
                    raise ValueError("Empty or invalid YAML file")

                self.get_logger().info(f"Loaded goals from {goals_path}")
                for color, data in goals.items():
                    self.get_logger().info(
                        f"  {color}: ({data['position']['x']}, {data['position']['y']})"
                    )
                return goals

        except Exception as e:
            self.get_logger().error(f"Error loading goals file: {e}")
            # Return empty dict - service will return failure
            return {}

    def _handle_get_goal_request(self, request, response):
        """Handle GetGoal service request"""
        time.sleep(10)
        color = request.color

        if color in self._goals:
            goal_data = self._goals[color]

            # Create PoseStamped message
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self._map_frame

            # Set position and orientation from stored data
            pose.pose.position.x = float(goal_data["position"]["x"])
            pose.pose.position.y = float(goal_data["position"]["y"])
            pose.pose.position.z = float(goal_data["position"]["z"])

            pose.pose.orientation.x = float(goal_data["orientation"]["x"])
            pose.pose.orientation.y = float(goal_data["orientation"]["y"])
            pose.pose.orientation.z = float(goal_data["orientation"]["z"])
            pose.pose.orientation.w = float(goal_data["orientation"]["w"])

            response.goal_pose = pose
            response.success = True
            response.message = f"Goal for color '{color}' provided successfully"

            self.get_logger().info(
                f"Provided goal for {color} at position ({pose.pose.position.x}, {pose.pose.position.y})"
            )
        else:
            response.success = False
            response.message = f"No goal defined for color '{color}'"

            self.get_logger().warn(f"No goal defined for color {color}")

        return response


def main(args=None):
    rclpy.init(args=args)

    goal_provider = GoalProviderService()

    try:
        rclpy.spin(goal_provider)
    except KeyboardInterrupt:
        pass
    finally:
        goal_provider.destroy_node()
        rclpy.shutdown()
