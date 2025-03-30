import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardControlPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_control_publisher')
        self.publisher_ = self.create_publisher(String, 'bat_direction', 10)
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.get_logger().info("Keyboard Control Node has started. Use arrow keys to move the bat.")

        # Variables to track steps and state
        self.step_counter = 0
        self.up_command_sent = False

        # Timer to check for step updates
        self.timer = self.create_timer(0.1, self.check_steps)

    def on_key_press(self, key):
        try:
            msg = String()
            if key == keyboard.Key.right:
                msg.data = "RIGHT"
            elif key == keyboard.Key.left:
                msg.data = "LEFT"
            elif key == keyboard.Key.up:
                msg.data = "UP"
                self.up_command_sent = True  # Set flag when UP is sent
                self.step_counter = 0       # Reset step counter
            # elif key == keyboard.Key.down:
            #     msg.data = "DOWN"
            else:
                return  # Ignore other keys

            # Publish the direction
            self.publisher_.publish(msg)
               
            self.get_logger().info(f"Published bat direction: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error in key press: {e}")

    def check_steps(self):
        # Check if UP was sent and increment the step counter
        if self.up_command_sent:
            self.step_counter += 1
            if self.step_counter >= 4:  # After 30 steps, send DOWN
                msg = String()
                msg.data = "DOWN"
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published bat direction: {msg.data} (auto after 30 steps)")
                self.up_command_sent = False  # Reset the flag


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()









# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String 
# import keyboard

# class KeyboardControlPublisher(Node):
#     def __init__(self):
#         super().__init__('keyboard_control_publisher')
#         self.publisher_ = self.create_publisher(String, 'bat_direction', 10)
#         self.timer = self.create_timer(0.1, self.publish_position)
#         self.get_logger().info("Keyboard Control Node has started. Use arrow keys to move the bat.")

#     # def publish_position(self):
#     #     # Check for keyboard input
#     #     key = keyboard.getKey()
#     #     if key == keyboard.RIGHT:  # Move motor up
#     #         print("Right key pressed")
#     #         self.bat_direction = "RIGHT"
#     #     elif key == keyboard.LEFT:  # Move motor down
#     #         print("Left key pressed")
#     #         self.bat_direction = "LEFT"
#     #     elif key == keyboard.UP:
#     #         print("Up key pressed")
#     #         self.bat_direction = "UP"

#     #     # Publish the new position
#     #     # self.publisher_.publish(self.bat_position)
#     #     self.publisher_.publish(self.bat_direction)
#     #     self.get_logger().info(f"Published bat position: {self.bat_direction}")
#     def publish_position(self):
#             # Create a message object
#             msg = String()

#             # Check for keyboard input
#             if keyboard.is_pressed('right'):  # Move bat right
#                 msg.data = "RIGHT"
#             elif keyboard.is_pressed('left'):  # Move bat left
#                 msg.data = "LEFT"
#             elif keyboard.is_pressed('up'):  # Move bat up
#                 msg.data = "UP"
#             elif keyboard.is_pressed('down'):  # Move bat down
#                 msg.data = "DOWN"
#             else:
#                 return  # Do not publish if no key is pressed

#             # Publish the direction
#             self.publisher_.publish(msg)
#             self.get_logger().info(f"Published bat direction: {msg.data}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardControlPublisher()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
    main()