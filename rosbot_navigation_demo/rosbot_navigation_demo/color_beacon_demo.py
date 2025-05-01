import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Quaternion, Point, Pose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from rosbot_interfaces.msg import ColorData
from std_msgs.msg import Header
import math
import tf_transformations


class ColorBeacon(Node):
    def __init__(self):
        super().__init__("color_beacon_demo")
        
        self._color_map = {
            "red": ColorData.RED,
            "green": ColorData.GREEN,
            "blue": ColorData.BLUE,
            "yellow": ColorData.YELLOW,
        }

        # Parameters for various colors
        self.declare_parameter("target_color", "yellow")
        self.declare_parameter("red_lower_hsv", [0, 30, 30])
        self.declare_parameter("red_upper_hsv", [15, 255, 255])
        self.declare_parameter("red_lower_hsv2", [160, 30, 30])
        self.declare_parameter("red_upper_hsv2", [179, 255, 255])

        # New color parameters
        self.declare_parameter("blue_lower_hsv", [100, 30, 30])
        self.declare_parameter("blue_upper_hsv", [130, 255, 255])
        self.declare_parameter("green_lower_hsv", [40, 30, 30])
        self.declare_parameter("green_upper_hsv", [80, 255, 255])
        self.declare_parameter("yellow_lower_hsv", [20, 30, 30])
        self.declare_parameter("yellow_upper_hsv", [35, 255, 255])

        self.declare_parameter("min_detection_area", 30)
        self.declare_parameter("rotation_speed", 0.3)
        self.declare_parameter("rotation_timer_period", 0.1)
        
        # Parameters for 3D pose estimation
        self.declare_parameter("cube_size_meters", 0.1)  # Actual size of the cube in meters
        self.declare_parameter("camera_focal_length_pixels", 500.0)  # Default focal length in pixels
        self.declare_parameter("debug_visualization", False)  # Enable/disable debug visualization

        # Get target color
        self._target_color = self.get_parameter("target_color").value

        # Get all color thresholds
        self._red_lower_hsv = np.array(self.get_parameter("red_lower_hsv").value)
        self._red_upper_hsv = np.array(self.get_parameter("red_upper_hsv").value)
        self._red_lower_hsv2 = np.array(self.get_parameter("red_lower_hsv2").value)
        self._red_upper_hsv2 = np.array(self.get_parameter("red_upper_hsv2").value)

        self._blue_lower_hsv = np.array(self.get_parameter("blue_lower_hsv").value)
        self._blue_upper_hsv = np.array(self.get_parameter("blue_upper_hsv").value)
        self._green_lower_hsv = np.array(self.get_parameter("green_lower_hsv").value)
        self._green_upper_hsv = np.array(self.get_parameter("green_upper_hsv").value)
        self._yellow_lower_hsv = np.array(self.get_parameter("yellow_lower_hsv").value)
        self._yellow_upper_hsv = np.array(self.get_parameter("yellow_upper_hsv").value)

        self._min_detection_area = self.get_parameter("min_detection_area").value
        self._rotation_speed = self.get_parameter("rotation_speed").value
        self._rotation_timer_period = self.get_parameter("rotation_timer_period").value
        
        # Get 3D pose estimation parameters
        self._cube_size_meters = self.get_parameter("cube_size_meters").value
        self._camera_focal_length_pixels = self.get_parameter("camera_focal_length_pixels").value
        self._debug_visualization = self.get_parameter("debug_visualization").value
        
        # Camera intrinsics (will be updated if we receive camera info)
        self._camera_matrix = np.array([
            [self._camera_focal_length_pixels, 0, 0],
            [0, self._camera_focal_length_pixels, 0],
            [0, 0, 1]
        ])
        self._dist_coeffs = np.zeros((5, 1))  # Default: no distortion
        self._camera_info_received = False

        # Rotation control flags
        self._should_rotate = True
        self._was_rotating = True  # Track previous rotation state
        self._last_detection_time = self.get_clock().now()
        self._detection_timeout = 1.0  # seconds
        self._target_color_detected = False

        # CV Bridge
        self._bridge = CvBridge()

        # Publishers
        self._color_pub = self.create_publisher(ColorData, "/color_beacon", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Add a separate pose publisher to ensure pose is available regardless of ColorData message format
        self._pose_pub = self.create_publisher(Pose, "/color_beacon_pose", 10)

        # Subscribers
        self._image_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self._image_callback, 10
        )
        
        # Subscribe to camera info to get accurate camera parameters
        self._camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self._camera_info_callback, 10
        )

        # Timer for persistent rotation
        self._rotation_timer = self.create_timer(
            self._rotation_timer_period, self._rotation_timer_callback
        )

        self.get_logger().info(
            f"ColorBeacon node initialized, looking for {self._target_color} color"
        )
    
    def _camera_info_callback(self, msg):
        """Process camera info to extract intrinsic parameters"""
        if not self._camera_info_received:
            # Extract camera matrix from camera info
            self._camera_matrix = np.array(msg.k).reshape(3, 3)
            
            # Extract distortion coefficients if available
            if hasattr(msg, 'distortion_model') and msg.distortion_model != "":
                self._dist_coeffs = np.array(msg.d)
            
            self._camera_info_received = True
            self.get_logger().info("Camera intrinsic parameters received")
            self.get_logger().debug(f"Camera matrix: {self._camera_matrix}")

    def _image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Create a debug visualization image if enabled
            if self._debug_visualization:
                debug_image = cv_image.copy()
            
            # Get image dimensions for camera frame positioning
            height, width, _ = cv_image.shape
            
            # Update camera matrix center point if not set properly
            if not self._camera_info_received:
                # Estimate camera center as the image center
                self._camera_matrix[0, 2] = width / 2.0
                self._camera_matrix[1, 2] = height / 2.0

            # Define detection functions for each color
            color_detectors = {
                "red": lambda img: self._detect_color(
                    img, self._red_lower_hsv, self._red_upper_hsv
                ),
                "blue": lambda img: self._detect_color(
                    img, self._blue_lower_hsv, self._blue_upper_hsv
                ),
                "green": lambda img: self._detect_color(
                    img, self._green_lower_hsv, self._green_upper_hsv
                ),
                "yellow": lambda img: self._detect_color(
                    img, self._yellow_lower_hsv, self._yellow_upper_hsv
                ),
            }

            # Special handling for red (which often requires two HSV ranges)
            if self._target_color == "red":
                color_detected, area, position_x, position_y, contour = self._detect_color_with_multiple_ranges(
                    cv_image, 
                    [self._red_lower_hsv, self._red_lower_hsv2], 
                    [self._red_upper_hsv, self._red_upper_hsv2]
                )
            # Only detect the target color
            elif self._target_color in color_detectors:
                detector = color_detectors[self._target_color]
                color_detected, area, position_x, position_y, contour = detector(cv_image)
            else:
                self.get_logger().warn(
                    f"Unknown target color: {self._target_color}", once=True
                )
                return

            if color_detected:
                # Estimate pose (position and orientation)
                position_3d, quaternion = self._estimate_pose_from_contour(
                    contour, width, height
                )
                
                # Add debug visualization if enabled
                if self._debug_visualization and contour is not None:
                    self._draw_pose_visualization(debug_image, contour, position_x, position_y, position_3d, quaternion)
                    cv2.imshow("Color Detection Debug", debug_image)
                    cv2.waitKey(1)
                
                # Create and publish ColorData message
                color_msg = ColorData()
                color_msg.header = Header()
                color_msg.header.stamp = self.get_clock().now().to_msg()
                color_msg.header.frame_id = "camera_color_optical_frame"  # Set to camera frame
                color_msg.color = self._color_map[self._target_color]
                color_msg.confidence = min(1.0, area / 10000)
                color_msg.area = area
                
                # Add position and orientation to the message (assuming ColorData has these fields)
                if hasattr(color_msg, 'position'):
                    color_msg.position.x = position_3d[0]
                    color_msg.position.y = position_3d[1]
                    color_msg.position.z = position_3d[2]
                    
                if hasattr(color_msg, 'orientation'):
                    color_msg.orientation.x = quaternion[0]
                    color_msg.orientation.y = quaternion[1]
                    color_msg.orientation.z = quaternion[2]
                    color_msg.orientation.w = quaternion[3]
                
                # Publish ColorData message
                self._color_pub.publish(color_msg)
                
                # Always publish the Pose message separately for compatibility
                pose_msg = Pose()
                pose_msg.position = Point(
                    x=position_3d[0], 
                    y=position_3d[1], 
                    z=position_3d[2]
                )
                pose_msg.orientation = Quaternion(
                    x=quaternion[0],
                    y=quaternion[1],
                    z=quaternion[2],
                    w=quaternion[3]
                )
                self._pose_pub.publish(pose_msg)

                # Throttle logging - only log once per second at most
                current_time = self.get_clock().now()
                if (
                    not hasattr(self, "_last_log_time")
                    or (current_time - self._last_log_time).nanoseconds / 1e9 > 1.0
                ):
                    pos_str = f"[{position_3d[0]:.3f}, {position_3d[1]:.3f}, {position_3d[2]:.3f}]"
                    quat_str = f"[{quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f}]"
                    self.get_logger().info(
                        f"{self._target_color.capitalize()} detected with confidence: {color_msg.confidence:.2f}, "
                        f"area: {area}, position: {pos_str}, "
                        f"orientation quaternion: {quat_str}",
                        once=True,
                    )
                    self._last_log_time = current_time

                # Update state for rotation control
                self._target_color_detected = True
                self._last_detection_time = current_time
                self._should_rotate = False
            else:
                # Check if it's been long enough since the last detection
                current_time = self.get_clock().now()
                time_since_detection = (
                    current_time - self._last_detection_time
                ).nanoseconds / 1e9

                if time_since_detection > self._detection_timeout:
                    if not self._should_rotate and self._target_color_detected:
                        self.get_logger().info(
                            f"Lost sight of {self._target_color}, enabling rotation"
                        )
                        self._target_color_detected = False
                    self._should_rotate = True

        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _detect_color_with_multiple_ranges(self, image, lower_hsv_list, upper_hsv_list):
        """
        Detect a color using multiple HSV ranges (useful for colors like red that wrap around the hue circle)
        Returns: (color_detected, area, center_x, center_y, contour)
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create a combined mask from all ranges
        combined_mask = np.zeros_like(hsv[:, :, 0])
        
        for lower_hsv, upper_hsv in zip(lower_hsv_list, upper_hsv_list):
            # Create mask for the current HSV range
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
            # Combine with the previous masks
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        # Process the combined mask
        return self._process_color_mask(combined_mask, image)

    def _detect_color(self, image, lower_hsv, upper_hsv):
        """
        Detect a specific color in the image using the provided HSV bounds
        Returns: (color_detected, area, center_x, center_y, contour)
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create mask for the color
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Process the mask
        return self._process_color_mask(mask, image)

    def _process_color_mask(self, mask, original_image):
        """
        Process a binary mask to determine if a color is detected and calculate orientation
        Returns: (color_detected, area, center_x, center_y, contour)
        """
        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours for orientation detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Try pixel-based detection first (more reliable for small areas)
        color_pixels = np.sum(mask) / 255  # Count white pixels in mask

        if color_pixels > self._min_detection_area:
            # Find the centroid of all colored pixels
            y_coords, x_coords = np.where(mask > 0)
            if len(y_coords) > 0:
                center_x = int(np.mean(x_coords))
                center_y = int(np.mean(y_coords))
                
                # If we have contours, use the largest one
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)

                    if area > self._min_detection_area:
                        return True, int(color_pixels), center_x, center_y, largest_contour
                
                # No valid contour, but we have pixel data
                return True, int(color_pixels), center_x, center_y, None

        # Find contours as backup method
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > self._min_detection_area:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    return True, int(area), center_x, center_y, largest_contour

        return False, 0, 0, 0, None

    def _estimate_pose_from_contour(self, contour, img_width, img_height):
        """
        Estimate 3D position and orientation from a contour
        Returns: (position_3d, quaternion)
        """
        # Default values
        default_position = (0.0, 0.0, 1.0)
        default_quaternion = (0.0, 0.0, 0.0, 1.0)  # Identity quaternion
        
        if contour is None:
            return default_position, default_quaternion
        
        # Get camera intrinsics
        fx = self._camera_matrix[0, 0]  # Focal length x
        fy = self._camera_matrix[1, 1]  # Focal length y
        cx = self._camera_matrix[0, 2]  # Principal point x
        cy = self._camera_matrix[1, 2]  # Principal point y
        
        # 1. Extract the basic contour properties
        # Find the center of the contour
        M = cv2.moments(contour)
        if M["m00"] == 0:
            return default_position, default_quaternion
            
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        
        # Find the minimum area rectangle that encloses the contour
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        # Get the dimensions of the rectangle
        (rect_center_x, rect_center_y), (width_px, height_px), angle_deg = rect
        
        # 2. Calculate depth (Z) using apparent size
        # Use the larger dimension of the rectangle for better stability
        apparent_size_px = max(width_px, height_px)
        
        # Avoid division by zero
        if apparent_size_px <= 0:
            return default_position, default_quaternion
            
        # Z = (actual size * focal length) / apparent size
        # This assumes the rectangle roughly corresponds to the actual size of the cube
        z = (self._cube_size_meters * fx) / apparent_size_px
        
        # 3. Calculate X and Y in camera frame
        # Origin is at camera center, X right, Y down, Z forward
        # Convert from pixel coordinates to camera coordinates
        x = (center_x - cx) * z / fx
        y = (center_y - cy) * z / fy
        
        # 4. Calculate orientation
        # OpenCV's minAreaRect returns angles in the range [-90, 0)
        # Need to adjust for camera frame convention
        if angle_deg < -45:
            angle_deg = 90 + angle_deg
        
        # Convert to radians for quaternion calculation
        angle_rad = math.radians(angle_deg)
        
        # In camera frame, cube rotation is primarily around Z axis
        # For a top-down view, this would be yaw
        roll = 0.0
        pitch = 0.0
        yaw = angle_rad
        
        # Convert to quaternion (ROS convention)
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        
        # Return the position and orientation
        return (x, y, z), quaternion

    def _draw_pose_visualization(self, image, contour, center_x, center_y, position_3d, quaternion):
        """Draw visualization of the pose estimation for debugging"""
        if contour is None:
            return
            
        # Draw the contour
        cv2.drawContours(image, [contour], 0, (0, 255, 0), 2)
        
        # Draw the center point
        cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Draw the minimum area rectangle
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(image, [box], 0, (255, 0, 0), 2)
        
        # Extract Euler angles from quaternion for visualization
        euler = tf_transformations.euler_from_quaternion(quaternion)
        
        # Show position and orientation info on the image
        pos_text = f"Pos: ({position_3d[0]:.2f}, {position_3d[1]:.2f}, {position_3d[2]:.2f})m"
        ori_text = f"Ori (RPY): ({math.degrees(euler[0]):.1f}, {math.degrees(euler[1]):.1f}, {math.degrees(euler[2]):.1f})°"
        
        cv2.putText(image, pos_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(image, ori_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw 3D axes projected onto the image to show orientation
        axis_length = 0.05  # 5cm axes
        self._draw_axes(image, position_3d, quaternion, axis_length)

    def _draw_axes(self, image, position, quaternion, length=0.1):
        """Draw 3D coordinate axes projected onto the image"""
        # Define the axes in 3D
        origin = np.array(position)
        
        # Create rotation matrix from quaternion
        rotation_mat = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        
        # Define the axis endpoints in 3D (camera frame)
        x_axis = origin + rotation_mat @ np.array([length, 0, 0])
        y_axis = origin + rotation_mat @ np.array([0, length, 0])
        z_axis = origin + rotation_mat @ np.array([0, 0, length])
        
        # Project the 3D points to 2D image coordinates
        fx = self._camera_matrix[0, 0]
        fy = self._camera_matrix[1, 1]
        cx = self._camera_matrix[0, 2]
        cy = self._camera_matrix[1, 2]
        
        # Function to project 3D point to 2D
        def project(point):
            # Only project if z > 0 (in front of camera)
            if point[2] <= 0:
                return None
            x = int(point[0] * fx / point[2] + cx)
            y = int(point[1] * fy / point[2] + cy)
            return (x, y)
        
        # Project origin and axes to image plane
        origin_2d = project(origin)
        x_axis_2d = project(x_axis)
        y_axis_2d = project(y_axis)
        z_axis_2d = project(z_axis)
        
        # Draw the axes if projection is valid
        if origin_2d:
            if x_axis_2d:
                cv2.line(image, origin_2d, x_axis_2d, (0, 0, 255), 2)  # X-axis in red
            if y_axis_2d:
                cv2.line(image, origin_2d, y_axis_2d, (0, 255, 0), 2)  # Y-axis in green
            if z_axis_2d:
                cv2.line(image, origin_2d, z_axis_2d, (255, 0, 0), 2)  # Z-axis in blue

    def _rotation_timer_callback(self):
        """Control robot rotation based on detection state"""
        # Only send commands when rotation state changes
        if self._should_rotate != self._was_rotating:
            twist = Twist()

            if self._should_rotate:
                # Start rotation
                twist.angular.z = self._rotation_speed
                self.get_logger().info(
                    f"Starting rotation at {self._rotation_speed} rad/s"
                )
            else:
                # Stop rotation
                twist.angular.z = 0.0
                self.get_logger().info(
                    f"Stopping rotation - {self._target_color} detected"
                )

            self._cmd_vel_pub.publish(twist)
            self._was_rotating = self._should_rotate
        elif self._should_rotate:
            # Continue rotation
            twist = Twist()
            twist.angular.z = self._rotation_speed
            self._cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Send command to stop the robot (called only during shutdown)"""
        twist = Twist()
        self._cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot stopped")


def main(args=None):
    rclpy.init(args=args)

    color_beacon = ColorBeacon()

    try:
        rclpy.spin(color_beacon)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        # Stop the robot when shutting down
        if "color_beacon" in locals():
            color_beacon.stop_robot()
            color_beacon.destroy_node()
        rclpy.shutdown()