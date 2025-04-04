import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from . import ball_detection as bd

class CameraFeedPublisher(Node):
    def __init__(self):
        super().__init__('camera_feed_publisher')

        self.h_publisher_ = self.create_publisher(String, 'h_position', 10)
        self.v_publisher_ = self.create_publisher(String, 'v_position', 10)
        
        self.get_logger().info("Camera Feed Node has started.")

        self.create_subscription(Image, '/robot/camera/image_color', self.camera_callback, 1)
       
    def camera_callback(self, msg):
        process_result = bd.detectBallWitContours(msg.data, msg.width, msg.height)
        if process_result is not None:
            position, radius = process_result
            self.get_logger().info(f"Ball detected at {position} with radius {radius}")
            # Publish the position and radius to the respective topics
            self.h_publisher_.publish(String(data=str(position[0])))
            self.v_publisher_.publish(String(data=str(position[1])))
        else:
            self.get_logger().info("No ball detected.")
            # Process the camera image if needed
            # For now, we will just print the image size
            # self.get_logger().info(f"Received camera image of size: {msg.width}x{msg.height}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraFeedPublisher()
    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()