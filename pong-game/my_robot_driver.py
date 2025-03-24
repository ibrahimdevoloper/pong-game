# def main():
#     print('Hi from pong-game.')


# if __name__ == '__main__':
#     main()

import rclpy
from geometry_msgs.msg import Twist

# HALF_DISTANCE_BETWEEN_WHEELS = 0.045
# WHEEL_RADIUS = 0.025

POSITION_HIGH = 0.2
POSITION_LOW = 0.02

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__bottom_bat_motor = self.__robot.getDevice('bottom_bat_motor')
        self.__bottom_bat_position = 0.11
        self.__bottom_bat_motor.setPosition(self.__bottom_bat_position)

        self.__bottom_rack = self.__robot.getDevice('bottom_rack')
        self.__bottom_rack_position = 0.01
        self.__bottom_rack.setPosition(self.__bottom_rack_position)

        self.__top_bat_motor = self.__robot.getDevice('top_bat_motor')
        self.__top_bat_position = 0.11
        self.__top_bat_motor.setPosition(self.__top_bat_position)

        self.__top_rack = self.__robot.getDevice('top_rack')
        self.__top_rack_position = 0.01
        self.__top_rack.setPosition(self.__top_rack_position)

        # self.__left_motor = self.__robot.getDevice('left wheel motor')
        # self.__right_motor = self.__robot.getDevice('right wheel motor')

        # self.__left_motor.setPosition(float('inf'))
        # self.__left_motor.setVelocity(0)

        # self.__right_motor.setPosition(float('inf'))
        # self.__right_motor.setVelocity(0)

        # self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        pass
        # self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # forward_speed = self.__target_twist.linear.x
        # angular_speed = self.__target_twist.angular.z

        # command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        # command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        # self.__left_motor.setVelocity(command_motor_left)
        # self.__right_motor.setVelocity(command_motor_right)
        i=0
        i+=0.01
        if self.__top_bat_position>POSITION_HIGH:
            self.__top_bat_position=POSITION_LOW
        else:
            self.__top_bat_position=POSITION_HIGH
        self.__top_bat_motor.setPosition(self.__top_bat_position)
