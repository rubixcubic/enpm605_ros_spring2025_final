import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class CubePublisher(Node):
    def __init__(self):
        super().__init__('cube_publisher')
        self._visualization_marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self._timer = self.create_timer(1.0, self.publish_cubes)


    def publish_cube(self, id, position, color, size, namespace='cubes'):
        marker = Marker()
        marker.header.frame_id = "odom"  # Or your desired frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]

        marker.color = ColorRGBA()
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        marker.lifetime.sec = 0  # Keep the marker persistent

        self._visualization_marker_publisher.publish(marker)

    def publish_cubes(self):
        self.publish_cube(
            id=0,
            position=[-1.0, 1.0, 0.1],
            color=[1.0, 0.0, 0.0, 1.0],  # Red
            size=[0.1, 0.1, 0.1],
            namespace='floating_cubes'
        )
        self.publish_cube(
            id=1,
            position=[-1.0, 0.5, 0.1],
            color=[0.0, 0.0, 1.0, 1.0],  # Blue
            size=[0.1, 0.1, 0.1],
            namespace='floating_cubes'
        )
        self.publish_cube(
            id=2,
            position=[-1.0, -1.0, 0.1],
            color=[0.0, 1.0, 0.0, 1.0],  # Green
            size=[0.1, 0.1, 0.1],
            namespace='floating_cubes'
        )
        self.publish_cube(
            id=3,
            position=[-1.0, -0.5, 0.1],
            color=[1.0, 1.0, 0.0, 1.0],  # Yellow
            size=[0.1, 0.1, 0.1],
            namespace='floating_cubes'
        )

def main(args=None):
    """
    Main function for the camera demo node.

    Initializes ROS2 communication, creates the node, and handles proper cleanup
    on exit. Includes comprehensive error handling.

    Args:
        args: Command line arguments passed to ROS2
    """
    try:
        # Initialize ROS2 communication
        rclpy.init(args=args)
        # Create and initialize the node
        node = CubePublisher()
        # Spin the node to process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle clean Ctrl+C exit gracefully
        print("Node stopped cleanly by user")
    except Exception as e:
        # Catch and report any other exceptions
        print(f"Error occurred: {e}")
    finally:
        # Cleanup resources properly
        if "node" in locals() and rclpy.ok():
            node.destroy_node()
        # Only call shutdown if ROS is still initialized
        if rclpy.ok():
            rclpy.shutdown()
