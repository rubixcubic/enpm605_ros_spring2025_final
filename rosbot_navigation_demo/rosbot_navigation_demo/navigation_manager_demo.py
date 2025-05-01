import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rosbot_interfaces.msg import ColorData
from rosbot_interfaces.srv import GetGoal
from rosbot_interfaces.action import MoveToGoal
import math
from rclpy.executors import MultiThreadedExecutor


class NavigationManager(Node):
    """
    A ROS 2 node that manages the robot's navigation based on detected colored beacons.

    This node subscribes to color detection messages, requests navigation goals
    from a service, and sends navigation commands to an action server. It handles
    cooldown periods between navigation requests, validates received goals, and
    manages the lifecycle of navigation actions, including feedback and timeouts.
    """

    def __init__(self):
        """
        Initializes the NavigationManager node.

        This includes declaring and retrieving parameters, initializing publishers,
        subscribers, action clients, and setting up internal state variables.
        """
        super().__init__("navigation_manager_demo")
        
        # Mapping between color string and color int
        self._color_map = {
            "red": ColorData.RED,
            "green": ColorData.GREEN,
            "blue": ColorData.BLUE,
            "yellow": ColorData.YELLOW,
        }

        # Declare and retrieve parameters
        self.declare_parameter("target_color", "red")
        self.declare_parameter("cooldown_seconds", 5.0)
        self.declare_parameter("distance_tolerance", 0.1)
        self.declare_parameter("angle_tolerance", 0.05)
        self.declare_parameter("navigation_timeout", 60.0)
        self.declare_parameter("min_confidence", 0.7)
        self.declare_parameter("expected_frame_id", "odom")
        self.declare_parameter("feedback_throttle", 2.0)
        self.declare_parameter("service_call_type", "async")
        
        

        self._target_color = self.get_parameter("target_color").value  # The color to trigger navigation.
        self._cooldown_duration = Duration(  # Minimum time between navigation requests.
            seconds=self.get_parameter("cooldown_seconds").value
        )
        self._distance_tolerance = self.get_parameter("distance_tolerance").value  # Tolerance for reaching the goal position (meters).
        self._angle_tolerance = self.get_parameter("angle_tolerance").value  # Tolerance for reaching the goal orientation (radians).
        self._navigation_timeout = self.get_parameter("navigation_timeout").value  # Maximum time allowed for a navigation action (seconds).
        self._min_confidence = self.get_parameter("min_confidence").value  # Minimum confidence level for a color detection to be considered.
        self._expected_frame_id = self.get_parameter("expected_frame_id").value  # Expected frame ID of the goal pose.
        self._feedback_throttle = self.get_parameter("feedback_throttle").value  # Minimum time between sending navigation feedback messages (seconds).
        
        self._service_call_type = self.get_parameter("service_call_type").value  # Type of service call (sync or async)
        

        # Internal state variables
        self._last_navigation_time = self.get_clock().now()  # Time of the last successful navigation.
        self._is_navigating = False  # Flag indicating if the robot is currently navigating.
        self._last_feedback_time = self.get_clock().now()  # Time of the last received navigation feedback.
        self._current_goal_handle = None  # Handle to the currently active navigation goal.
        self._navigation_timer = None  # Timer for tracking navigation timeout.

        # Callback group for non-blocking calls
        self._callback_group = ReentrantCallbackGroup()

        # Subscriber to color detection messages
        self._color_sub = self.create_subscription(
            ColorData,
            "/color_beacon",
            self._color_callback,
            10,
            callback_group=self._callback_group,
        )

        # Client for the GetGoal service
        self._goal_client_async = self.create_client(
            GetGoal, "/get_goal"
        )
        
        self._goal_client_sync = self.create_client(
            GetGoal, "/get_goal"
        )

        # Action client for the MoveToGoal action
        self._nav_client = ActionClient(
            self, MoveToGoal, "/move_to_goal", callback_group=self._callback_group
        )

        # Wait for required servers to become available
        self._wait_for_servers()

        self.get_logger().info("navigation_manager_demo node initialized")
        self.get_logger().info(
            f"Target color: {self._target_color}, Min confidence: {self._min_confidence}"
        )

    def _wait_for_servers(self):
        """
        Waits for the GetGoal service and MoveToGoal action server to be available.

        Logs informative messages while waiting and exits if ROS 2 is shut down.
        """
        self.get_logger().info("Waiting for GetGoal service...")
        while not self._goal_client_async.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for service")
                return
            self.get_logger().info("Still waiting for GetGoal service...")

        self.get_logger().info("Waiting for MoveToGoal action server...")
        while not self._nav_client.wait_for_server(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for action server")
                return
            self.get_logger().info("Still waiting for MoveToGoal action server...")

    def _color_callback(self, msg):
        """
        Callback function for processing received ColorData messages.

        Triggers navigation if the detected color matches the target color and
        the cooldown period has elapsed.

        Args:
            msg (ColorData): The received color data message.
        """
        if msg.color != self._color_map[self._target_color] or self._is_navigating:
            return

        # Consider putting this back: This comment indicates a potentially useful check.
        # if msg.confidence < self._min_confidence:
        #     self.get_logger().debug(f"Low confidence for {msg.color}: {msg.confidence}")
        #     return

        current_time = self.get_clock().now()
        if current_time - self._last_navigation_time > self._cooldown_duration:
            self.get_logger().info(
                f"Detected {msg.color} with confidence {msg.confidence}"
            )
            self._request_get_goal(msg.color)

    def _request_get_goal(self, color):
        """
        Requests a navigation goal from the GetGoal service for the given color.

        Sets the internal navigation flag to True and sends an asynchronous request
        to the service. The response is handled by the _goal_response_callback.

        Args:
            color (str): The color for which to request a navigation goal.
        """
        self._is_navigating = True
        self.get_logger().info(f"Requesting goal for color: {color}")

        request = GetGoal.Request()
        request.color = color
        if self._service_call_type == "async":
            future = self._goal_client_async.call_async(request) # Make the asynchronous call
            future.add_done_callback(self._goal_response_callback)
            self.get_logger().info("======> After async service call")
        elif self._service_call_type == "sync":
            result = self._goal_client_sync.call(request) # Make the synchronous call
            self.get_logger().info("======> After sync service call")
            if result.success:
                self.get_logger().info("Goal was successful")
                self._send_navigation_goal(result.goal_pose)
            else:
                self.get_logger().warn(f"Goal failed: {result.message}")

        else:
            self.get_logger().error(f"Unknown service call type: {self._service_call_type}")


    def _goal_response_callback(self, future):
        """
        Callback function to handle the response from the GetGoal service.

        If the service call is successful and the received goal pose is valid,
        it initiates the navigation action. Otherwise, it logs an error and resets
        the navigation flag.

        Args:
            future (rclpy.client.Future): The future object representing the result
                of the service call.
        """
        try:
            response = future.result()
            if response.success and self._validate_goal_pose(response.goal_pose):
                self.get_logger().info(
                    f"Goal pose received in frame {response.goal_pose.header.frame_id}"
                )
                self._send_navigation_goal(response.goal_pose)
            else:
                self.get_logger().error(f"Failed to get valid goal: {response.message}")
                self._is_navigating = False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.get_logger().error(f"Future exception: {future.exception()}")
            self._is_navigating = False

    def _validate_goal_pose(self, pose):
        """
        Validates the received goal pose.

        Checks if the frame ID matches the expected frame ID and if the pose
        contains any NaN values. It also normalizes the quaternion if its norm
        is significantly different from 1.

        Args:
            pose (geometry_msgs.msg.PoseStamped): The goal pose to validate.

        Returns:
            bool: True if the goal pose is valid, False otherwise.
        """
        if pose.header.frame_id != self._expected_frame_id:
            self.get_logger().warn(f"Unexpected frame: {pose.header.frame_id}")

        components = [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        if any(math.isnan(x) for x in components):
            self.get_logger().error("Goal pose contains NaN values")
            return False

        quat_norm = (
            pose.pose.orientation.x**2
            + pose.pose.orientation.y**2
            + pose.pose.orientation.z**2
            + pose.pose.orientation.w**2
        )
        if abs(quat_norm - 1.0) > 0.01:
            self.get_logger().warn(
                f"Non-normalized quaternion (norm={quat_norm:.3f}), normalizing"
            )
            norm = math.sqrt(quat_norm)
            pose.pose.orientation.x /= norm
            pose.pose.orientation.y /= norm
            pose.pose.orientation.z /= norm
            pose.pose.orientation.w /= norm

        return True

    def _send_navigation_goal(self, pose):
        """
        Sends a navigation goal to the MoveToGoal action server.

        Creates a MoveToGoal goal message with the target pose and tolerance
        parameters, and sends it to the action server with feedback and result
        callbacks.

        Args:
            pose (geometry_msgs.msg.PoseStamped): The target goal pose.
        """
        goal_msg = MoveToGoal.Goal()
        goal_msg.target_pose = pose
        goal_msg.distance_tolerance = self._distance_tolerance
        goal_msg.angle_tolerance = self._angle_tolerance

        self.get_logger().info(
            f"Sending navigation goal: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
        )

        send_goal_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self._navigation_goal_response_callback)

    def _navigation_goal_response_callback(self, future):
        """
        Callback function to handle the response after sending a navigation goal.

        If the goal is accepted by the action server, it logs a message and starts
        a timeout timer. Otherwise, it logs an error and resets the navigation flag.

        Args:
            future (rclpy.client.Future): The future object representing the result
                of sending the goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server")
            self._is_navigating = False
            return

        self._current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted by action server")

        # Start timeout timer only after goal is accepted
        self._navigation_timer = self.create_timer(
            self._navigation_timeout, self._navigation_timeout_callback
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result_callback)

    def _navigation_feedback_callback(self, feedback_msg):
        """
        Callback function to process feedback from the MoveToGoal action server.

        Logs the remaining distance and angle to the goal, throttled by the
        _feedback_throttle parameter.

        Args:
            feedback_msg (MoveToGoal.Feedback): The feedback message received
                from the action server.
        """
        feedback = feedback_msg.feedback
        current_time = self.get_clock().now()
        if (
            current_time - self._last_feedback_time
        ).nanoseconds / 1e9 >= self._feedback_throttle:
            self.get_logger().info(
                f"Progress: {feedback.distance_remaining:.2f}m left, {feedback.angle_remaining:.2f} rad"
            )
            self._last_feedback_time = current_time

    def _navigation_result_callback(self, future):
        """
        Callback function to process the result of the MoveToGoal action.

        Logs whether the navigation was successful or failed, and then calls
        _cleanup_after_navigation to reset the node's state.

        Args:
            future (rclpy.client.Future): The future object representing the result
                of the action.
        """
        try:
            result = future.result().result
            if result.success:
                self.get_logger().info(f"Navigation completed: {result.message}")
            else:
                self.get_logger().error(f"Navigation failed: {result.message}")
        except Exception as e:
            self.get_logger().error(f"Navigation result error: {e}")

        self._cleanup_after_navigation()

    def _navigation_timeout_callback(self):
        """
        Callback function to handle navigation timeouts.

        If the navigation is still active when the timeout occurs, it logs an
        error and attempts to cancel the current goal. Finally, it calls
        _cleanup_after_navigation.
        """
        if self._is_navigating:
            self.get_logger().error(
                f"Navigation timed out after {self._navigation_timeout} seconds"
            )
            if getattr(self._current_goal_handle, "accepted", False):
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._cancel_done_callback)

        self._cleanup_after_navigation()

    def _cancel_done_callback(self, future):
        """
        Callback function to handle the result of a cancel goal request.

        Logs whether the cancellation was successful or failed.

        Args:
            future (rclpy.client.Future): The future object representing the result
                of the cancel goal request.
        """
        try:
            result = future.result()
            if result.return_code == result.SUCCEEDED:
                self.get_logger().info("Navigation goal successfully canceled")
            else:
                self.get_logger().warn("Failed to cancel navigation goal")
        except Exception as e:
            self.get_logger().error(f"Cancel error: {e}")

    def _cleanup_after_navigation(self):
        """
        Resets the node's state after a navigation attempt (success, failure, or timeout).

        This includes canceling the navigation timer, resetting the navigation flag,
        updating the last navigation time, and clearing the current goal handle.
        """
        if self._navigation_timer is not None:
            self._navigation_timer.cancel()
            self._navigation_timer = None

        self._is_navigating = False
        self._last_navigation_time = self.get_clock().now()
        self._current_goal_handle = None


def main(args=None):
    """
    Main function for the navigation_manager_demo node.

    Initializes ROS 2, creates the NavigationManager node, and spins it to
    process incoming messages and callbacks. Handles keyboard interrupts for
    graceful shutdown.

    Args:
        args (list, optional): Command line arguments passed to the node.
            Defaults to None.
    """
    rclpy.init(args=args)
    node = NavigationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down from keyboard interrupt")
    finally:
        
        if node._navigation_timer is not None:
            node._navigation_timer.cancel()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()