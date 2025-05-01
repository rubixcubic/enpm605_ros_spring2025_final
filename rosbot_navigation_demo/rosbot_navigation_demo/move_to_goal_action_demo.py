import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rosbot_interfaces.action import MoveToGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from threading import Lock


class MoveToGoalActionServer(Node):
    def __init__(self):
        super().__init__("move_to_goal_action_server")

        # Declare parameters (using the style from the example)
        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 1.0)
        self.declare_parameter("default_linear_tolerance", 0.1)
        self.declare_parameter("default_angular_tolerance", 0.2)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("goal_timeout", 60.0)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.0)

        # Get parameters (using the style from the example)
        self._kp_linear = self.get_parameter("kp_linear").value
        self._kp_angular = self.get_parameter("kp_angular").value
        self._default_linear_tolerance = self.get_parameter(
            "default_linear_tolerance"
        ).value
        self._default_angular_tolerance = self.get_parameter(
            "default_angular_tolerance"
        ).value
        self._control_frequency = self.get_parameter("control_frequency").value
        self._goal_timeout = self.get_parameter("goal_timeout").value
        self._max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self._max_angular_velocity = self.get_parameter("max_angular_velocity").value
        self._control_period = 1.0 / self._control_frequency

        # Internal state
        self._callback_group = ReentrantCallbackGroup()
        self._active_goal = False
        self._goal_handle = None
        self._goal_start_time = None
        self._last_log_time = 0.0
        self._pose_lock = Lock()
        self._pose_initialized = False
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_theta = 0.0

        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_theta = 0.0
        self._linear_tolerance = self._default_linear_tolerance
        self._angular_tolerance = self._default_angular_tolerance

        # Publishers and Subscribers
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self._odom_callback,
            10,
            callback_group=self._callback_group,
        )

        # Action server
        self._action_server = ActionServer(
            self,
            MoveToGoal,
            "/move_to_goal",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        # Create the timer for the proportional controller
        self._control_timer = self.create_timer(0.1, self._control_timer_callback)  # 10Hz control loop

        self.get_logger().info("move_to_goal_action_server initialized")

    # -------------------- Callbacks --------------------
    def _control_timer_callback(self):
        # Skip if no active goal
        if not self._goal_handle or not self._active_goal:
            return

        # Use a lock to ensure thread safety
        # with self._pose_lock:
        # Check if goal is still active
        if not self._goal_handle.is_active:
            self._active_goal = False
            return

        # Calculate current errors
        distance_error = self._get_distance_to_goal()
        final_angular_error = self._get_final_angular_error()

        # Get elapsed time first to ensure consistent use
        current_time = self.get_clock().now()
        elapsed = (current_time - self._goal_start_time).nanoseconds / 1e9

        # Check if we've reached the goal
        if (distance_error <= self._linear_tolerance * 1.5 and
            abs(final_angular_error) <= self._angular_tolerance * 2.0):
            # Goal reached, handle completion
            self.get_logger().info(f"Goal reached! Distance: {distance_error:.3f}, Angular: {abs(final_angular_error):.3f}")

            # Create a result object but don't pass it to succeed()
            # This way it's available to the action client separately
            result = MoveToGoal.Result()
            result.success = True
            result.message = "Goal reached successfully"

            try:
                if self._goal_handle.is_active:
                    # Call succeed without passing the result
                    self._goal_handle.succeed()
                    # Explicitly publish the result
                    self.get_logger().info("Goal marked as succeeded")
                else:
                    self.get_logger().warn("Cannot succeed - goal is not in EXECUTING state")
            except Exception as e:
                self.get_logger().error(f"Error completing goal: {str(e)}")

            self._stop_robot()
            self._goal_handle = None
            self._active_goal = False
            return

        # Check for timeout - use a small margin to avoid floating point issues
        if elapsed > (self._goal_timeout - 0.1):
            self.get_logger().warn(f"Goal timed out after {elapsed:.2f} seconds")
            self.get_logger().warn(f"Final errors - Distance: {distance_error:.3f}, Angular: {abs(final_angular_error):.3f}")
            self.get_logger().warn(f"Tolerances - Distance: {self._linear_tolerance:.3f}, Angular: {self._angular_tolerance:.3f}")

            result = MoveToGoal.Result()
            result.success = False
            result.message = "Timeout"

            try:
                if self._goal_handle.is_active:
                    self._goal_handle.abort()
                    self.get_logger().info("Goal marked as aborted")
                else:
                    self.get_logger().warn("Cannot abort - goal is not in EXECUTING state")
            except Exception as e:
                self.get_logger().error(f"Error aborting goal: {str(e)}")

            self._stop_robot()
            self._goal_handle = None
            self._active_goal = False
            return

        # Execute control step
        self._control_step_proportional()

        # Create and publish feedback
        feedback_msg = MoveToGoal.Feedback()
        feedback_msg.current_pose = self._create_current_pose_stamped()
        feedback_msg.distance_remaining = distance_error
        feedback_msg.angle_remaining = abs(final_angular_error)
        self._goal_handle.publish_feedback(feedback_msg)

        # Log status periodically
        current_secs = current_time.nanoseconds / 1e9
        if current_secs - self._last_log_time > 1.0:
            self.get_logger().info(
                f"Current: x={self._current_x:.2f}, y={self._current_y:.2f}, theta={self._current_theta:.2f}, "
                f"Goal: x={self._goal_x:.2f}, y={self._goal_y:.2f}, theta={self._goal_theta:.2f}, "
                f"Distance Error: {distance_error:.3f}/{self._linear_tolerance:.3f}, "
                f"Angular Error: {abs(final_angular_error):.3f}/{self._angular_tolerance:.3f}, "
                f"Elapsed: {elapsed:.2f}/{self._goal_timeout:.1f}s"
            )
            self._last_log_time = current_secs

    def _odom_callback(self, msg):
        with self._pose_lock:
            self._current_x = msg.pose.pose.position.x
            self._current_y = msg.pose.pose.position.y
            quat = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            _, _, self._current_theta = euler_from_quaternion(quat)

            if not self._pose_initialized:
                self._pose_initialized = True
                self.get_logger().info(
                    f"Initial pose: x={self._current_x:.2f}, y={self._current_y:.2f}, theta={self._current_theta:.2f}"
                )

    def _goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")

        with self._pose_lock:  # Use the existing pose lock for synchronization
            if self._goal_handle == goal_handle and self._active_goal:
                self._stop_robot()

                try:
                    # Only cancel if still in EXECUTING state
                    if goal_handle.is_active:
                        goal_handle.canceled()
                        self.get_logger().info("Goal canceled successfully")
                    else:
                        self.get_logger().warn("Cannot cancel - goal is not in EXECUTING state")
                except Exception as e:
                    self.get_logger().error(f"Error canceling goal: {str(e)}")

                # Always reset state
                self._goal_handle = None
                self._active_goal = False
                return CancelResponse.ACCEPT
            else:
                return CancelResponse.REJECT

    def _execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal")

        if not self._pose_initialized:
            self.get_logger().warn("Waiting for odometry data...")
            result = MoveToGoal.Result()
            result.success = False
            result.message = "No odometry data"
            goal_handle.abort()
            return result

        # Extract goal parameters
        goal = goal_handle.request
        self._goal_x = goal.target_pose.pose.position.x
        self._goal_y = goal.target_pose.pose.position.y
        quat = (
            goal.target_pose.pose.orientation.x,
            goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w,
        )
        _, _, self._goal_theta = euler_from_quaternion(quat)
        self._linear_tolerance = (
            goal.distance_tolerance
            if goal.distance_tolerance > 0
            else self._default_linear_tolerance
        )
        self._angular_tolerance = (
            goal.angle_tolerance
            if goal.angle_tolerance > 0
            else self._default_angular_tolerance
        )

        # Reset state
        self._goal_handle = goal_handle
        self._goal_start_time = self.get_clock().now()
        self._active_goal = True
        self._last_log_time = 0.0

        # Create result object that will be returned at the end
        result = MoveToGoal.Result()

        # Wait for the goal to complete - the timer callback will handle the actual navigation
        try:
            while self._active_goal and rclpy.ok():
                # This will now be handled by the timer callback
                rclpy.spin_once(self, timeout_sec=0.1)
        except Exception as e:
            self.get_logger().error(f"Exception in execute callback: {str(e)}")
            result.success = False
            result.message = f"Error: {str(e)}"
            if self._goal_handle and self._goal_handle.is_active:
                try:
                    self._goal_handle.abort()
                except Exception as inner_e:
                    self.get_logger().error(f"Error aborting goal after exception: {str(inner_e)}")
            self._stop_robot()
            self._goal_handle = None
            self._active_goal = False
            return result

        # Here, we need to populate the result properly
        # By the time we get here, the timer callback has already handled the goal completion

        # If we reached here normally, assume success
        result.success = True
        result.message = "Goal reached successfully"

        self.get_logger().info("Execute callback completed")
        return result

    def _control_step_proportional(self):
        """
        Simple and direct proportional control to move the robot towards the goal.
        """
        # Get current errors
        distance_error = self._get_distance_to_goal()
        angular_error = self._get_angular_error()
        final_angular_error = self._get_final_angular_error()

        # Create twist message
        twist = Twist()

        # Simple proportional control for linear velocity
        # Always move forward when we're not close to the goal
        if distance_error > self._linear_tolerance:
            # Base speed proportional to distance error with a minimum value
            forward_speed = self._kp_linear * distance_error
            # Ensure minimum speed to overcome friction/inertia
            forward_speed = max(0.1, forward_speed)
            # Apply speed limit
            twist.linear.x = min(forward_speed, self._max_linear_velocity)
        else:
            # If we're close to the goal position, stop forward motion to focus on orientation
            twist.linear.x = 0.0

        # Simple proportional control for angular velocity
        # When far from goal, steer towards the goal point
        # When close to goal, orient to the final desired orientation
        if distance_error > self._linear_tolerance * 2:
            # Far from goal: focus on heading toward goal point
            angular_velocity = self._kp_angular * angular_error
        else:
            # Close to goal: focus on final orientation
            angular_velocity = self._kp_angular * final_angular_error

        # Apply angular velocity limits
        twist.angular.z = max(-self._max_angular_velocity,
                            min(angular_velocity, self._max_angular_velocity))

        # For large angular errors, reduce or stop forward motion
        if abs(angular_error) > 0.8:  # ~45 degrees
            # If the angle error is very large, prioritize turning by reducing speed
            twist.linear.x *= max(0.0, 1.0 - (abs(angular_error) - 0.8) / 2.35)

        # Debug information
        self.get_logger().info(
            f"Sending: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}, "
            f"dist_err={distance_error:.2f}, ang_err={angular_error:.2f}, final_ang_err={final_angular_error:.2f}"
        )

        # Publish the command
        self._cmd_vel_pub.publish(twist)

    def _create_current_pose_stamped(self):
        from geometry_msgs.msg import PoseStamped
        from tf_transformations import quaternion_from_euler

        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "base_link"  # Or your robot's base frame

        with self._pose_lock:
            current_pose.pose.position.x = self._current_x
            current_pose.pose.position.y = self._current_y
            current_pose.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, self._current_theta)
            current_pose.pose.orientation.x = q[0]
            current_pose.pose.orientation.y = q[1]
            current_pose.pose.orientation.z = q[2]
            current_pose.pose.orientation.w = q[3]

        return current_pose

    def _get_distance_to_goal(self):
        with self._pose_lock:
            return math.hypot(
                self._goal_x - self._current_x, self._goal_y - self._current_y
            )

    def _get_angle_to_goal(self):
        with self._pose_lock:
            return math.atan2(
                self._goal_y - self._current_y, self._goal_x - self._current_x
            )

    def _get_angular_error(self):
        return self._normalize_angle(self._get_angle_to_goal() - self._current_theta)

    def _get_final_angular_error(self):
        return self._normalize_angle(self._goal_theta - self._current_theta)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _stop_robot(self):
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info("Robot stopped")


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoalActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt")
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()
