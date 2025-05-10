import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from . import ball_detection as bd

class CameraFeedPublisher(Node):
    def __init__(self):
        super().__init__('camera_feed_publisher')

        # self.h_publisher_ = self.create_publisher(String, 'h_position', 10)
        # self.v_publisher_ = self.create_publisher(String, 'v_position', 10)
        self.v_publisher_ = self.create_publisher(Point, 'position_point', 10)
        self.estimated_speed_publisher = self.create_publisher(String, 'estimated_speed', 10)

        
        self.get_logger().info("Camera Feed Node has started.")

        self.create_subscription(Image, '/robot/camera/image_color', self.camera_callback, 1)

        self.previous_detected_position = 0.11
        self.time_step = 0.1  # Default time step (in seconds)
       
    def camera_callback(self, msg):
        process_result = bd.detectBallWitContours(msg.data, msg.width, msg.height)
        center, newPositionInCm = bd.detectBat(msg.data, msg.width, msg.height)
        speed = abs(newPositionInCm - self.previous_detected_position)/self.time_step*100 # cm/s
        self.previous_detected_position = newPositionInCm
        if speed>0:
            self.get_logger().info(f"Speed: {speed} cm/s")
            # Publish the estimated speed
            self.estimated_speed_publisher.publish(String(data=str(speed)))
        if process_result is not None:
            position, radius = process_result
            # self.get_logger().info(f"Ball detected at {position} with radius {radius}")
            # Publish the position and radius to the respective topics
            # self.h_publisher_.publish(String(data=str(position[0])))
            # self.v_publisher_.publish(String(data=str(position[1])))
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = radius
            # self.get_logger().info(f"Publishing Point: {point}")
            # self.get_logger().info(f"Publishing Point: x={point.x}, y={point.y}, z={point.z}")

            self.v_publisher_.publish(point)
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