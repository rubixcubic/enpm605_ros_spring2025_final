import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class ProportionalController(Node):
    def __init__(self):
        super().__init__("rosbot_p_controller")

        # Declare parameters
        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 1.0)
        self.declare_parameter("goal_x", 2.0)
        self.declare_parameter("goal_y", 1.0)
        self.declare_parameter("goal_theta", math.pi / 2.0)
        self.declare_parameter("linear_tolerance", 0.1)
        self.declare_parameter("angular_tolerance", 0.05)
        self.declare_parameter("control_frequency", 20.0)

        # Get parameters
        self._kp_linear = self.get_parameter("kp_linear").value
        self._kp_angular = self.get_parameter("kp_angular").value
        self._goal_x = self.get_parameter("goal_x").value
        self._goal_y = self.get_parameter("goal_y").value
        self._goal_theta = self.get_parameter("goal_theta").value
        self._linear_tolerance = self.get_parameter("linear_tolerance").value
        self._angular_tolerance = self.get_parameter("angular_tolerance").value
        self._control_frequency = self.get_parameter("control_frequency").value

        # Initialize pose variables
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta = 0.0
        self._pose_initialized = False

        # Publishers and subscribers
        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odometry_subscriber = self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_callback, 10
        )

        # Control timer
        self._timer = self.create_timer(
            1.0 / self._control_frequency, self._control_callback
        )

        self.get_logger().info(
            f"Proportional Controller initialized with goal: x={self._goal_x:.2f}, y={self._goal_y:.2f}, theta={self._goal_theta:.2f}"
        )
        self.get_logger().info(
            f"Linear gain: Kp={self._kp_linear}, Angular gain: Kp={self._kp_angular}"
        )

    def set_goal(self, x, y, theta):
        """
        Sets the goal pose.

        Args:
            x (float): Goal x-coordinate.
            y (float): Goal y-coordinate.
            theta (float): Goal orientation in radians.
        """
        self._goal_x = x
        self._goal_y = y
        self._goal_theta = theta
        self.get_logger().info(
            f"Goal updated to: x={self._goal_x:.2f}, y={self._goal_y:.2f}, theta={self._goal_theta:.2f}"
        )

    def _odom_callback(self, msg):
        """
        Callback function for odometry data. Updates the current pose.

        Args:
            msg (Odometry): Odometry message.
        """
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        _, _, self._current_theta = euler_from_quaternion(quaternion)

        if not self._pose_initialized:
            self._pose_initialized = True
            self.get_logger().info(
                f"Initial pose: x={self._current_x:.2f}, y={self._current_y:.2f}, theta={self._current_theta:.2f}"
            )

    def _normalize_angle(self, angle):
        """
        Normalize an angle to be within -pi to pi.

        Args:
            angle (float): Angle in radians.

        Returns:
            float: Normalized angle in radians.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _get_distance_to_goal(self):
        """
        Calculates the Euclidean distance to the goal.

        Returns:
            float: Distance to the goal.
        """
        return math.sqrt(
            (self._goal_x - self._current_x) ** 2
            + (self._goal_y - self._current_y) ** 2
        )

    def _get_angle_to_goal(self):
        """
        Calculates the angle to the goal in the global frame.

        Returns:
            float: Angle to the goal.
        """
        return math.atan2(
            self._goal_y - self._current_y, self._goal_x - self._current_x
        )

    def _get_angular_error(self):
        """
        Calculates the angular error between the current orientation and the
        orientation needed to face the goal.

        Returns:
            float: Angular error.
        """
        angle_to_goal = self._get_angle_to_goal()
        return self._normalize_angle(angle_to_goal - self._current_theta)

    def _get_final_angular_error(self):
        """
        Calculates the angular error between the current orientation and the
        final goal orientation.

        Returns:
            float: Final angular error.
        """
        return self._normalize_angle(self._goal_theta - self._current_theta)

    def _control_callback(self):
        """
        Periodic callback function for the control loop.
        """
        if not self._pose_initialized:
            self.get_logger().warn("Waiting for odometry data...")
            return

        distance_error = self._get_distance_to_goal()
        angular_error = self._get_angular_error()
        final_angular_error = self._get_final_angular_error()

        twist = Twist()

        # Combined movement - always try to move and turn simultaneously
        if distance_error > self._linear_tolerance:
            # Calculate linear velocity based on distance error
            twist.linear.x = self._kp_linear * distance_error

            # While moving, blend the angular control between:
            # - Heading to goal position when far
            # - Gradually adjusting to final orientation as we get closer
            position_weight = min(1.0, distance_error / 0.5)  # Full weight until 0.5m
            orientation_weight = 1.0 - position_weight

            # Blend the two angular errors
            blended_error = (position_weight * angular_error) + (
                orientation_weight * final_angular_error
            )
            twist.angular.z = self._kp_angular * blended_error

            self.get_logger().debug(
                f"Moving: distance={distance_error:.2f}, blended_angle={blended_error:.2f}, "
                f"weights: pos={position_weight:.2f}, orient={orientation_weight:.2f}"
            )
        else:
            # At goal position, only adjust orientation
            twist.linear.x = 0.0
            twist.angular.z = self._kp_angular * final_angular_error
            self.get_logger().debug(
                f"At goal, adjusting orientation: error={final_angular_error:.2f}"
            )

            # If we're close enough to final orientation, we're done
            if abs(final_angular_error) <= self._angular_tolerance:
                twist.angular.z = 0.0
                self.get_logger().info("Goal reached successfully!")

        # Apply velocity limits
        twist.linear.x = max(-0.5, min(twist.linear.x, 0.5))  # Limit to ±0.5 m/s
        twist.angular.z = max(-1.0, min(twist.angular.z, 1.0))  # Limit to ±1.0 rad/s

        # Publish velocity command
        self._cmd_vel_publisher.publish(twist)

        # Log progress periodically (about once per second)
        if self.get_clock().now().nanoseconds % int(1e9) < int(
            1e9 / self._control_frequency
        ):
            self.get_logger().info(
                f"Current: x={self._current_x:.2f}, y={self._current_y:.2f}, theta={self._current_theta:.2f}, "
                f"distance={distance_error:.2f}, angle_error={angular_error:.2f}"
            )

    def stop_robot(self):
        """Send a zero velocity command to stop the robot."""
        twist = Twist()
        self._cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot stopped")


def main(args=None):
    """
    Main function for the TurtleBot controller node.

    Args:
        args: Command line arguments passed to ROS2
    """
    rclpy.init(args=args)
    controller = None

    try:
        # Create and initialize the node
        controller = ProportionalController()

        # Spin the node to process callbacks
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nNode stopped by user")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        # Cleanup resources
        if controller is not None:
            controller.stop_robot()
            controller.destroy_node()
        rclpy.shutdown()
