# def main():
#     print('Hi from pong_game.')


# if __name__ == '__main__':
#     main()

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# HALF_DISTANCE_BETWEEN_WHEELS = 0.045
# WHEEL_RADIUS = 0.025

POSITION_HIGH = 0.19
POSITION_LOW = 0.03
POSITION_STEP = 0.01

RACK_POSITION_HIGH = 0.025
RACK_POSITION_LOW = 0.01

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__bottom_bat_motor = self.__robot.getDevice('bottom_bat_motor')
        self.__bottom_bat_position = 0.11
        self.__bottom_bat_motor.setPosition(self.__bottom_bat_position)

        self.__bottom_rack = self.__robot.getDevice('bottom_rack')
        self.__bottom_rack_position = False
        self.__bottom_rack.setPosition(RACK_POSITION_LOW)

        self.__top_bat_motor = self.__robot.getDevice('top_bat_motor')
        self.__top_bat_position = 0.11
        self.__top_bat_motor.setPosition(self.__top_bat_position)

        self.__top_rack = self.__robot.getDevice('top_rack')
        self.__top_rack_position = False
        self.__top_rack.setPosition(RACK_POSITION_LOW)

        # self.__left_motor = self.__robot.getDevice('left wheel motor')
        # self.__right_motor = self.__robot.getDevice('right wheel motor')

        # self.__left_motor.setPosition(float('inf'))
        # self.__left_motor.setVelocity(0)

        # self.__right_motor.setPosition(float('inf'))
        # self.__right_motor.setVelocity(0)

        # self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        # self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(String, 'bat_direction', self.__bat_direction_callback, 1)

    def __cmd_vel_callback(self, twist):
        pass
        # self.__target_twist = twist
    def __bat_direction_callback(self, msg):
        bat_direction = msg.data
        print('__bat_direction_callback:', bat_direction)
        if bat_direction == "UP":
            self.__bottom_rack_position = True
        if bat_direction == "DOWN":
            self.__bottom_rack_position = False
        elif bat_direction == "RIGHT":
            self.__bottom_bat_position += POSITION_STEP
            if self.__bottom_bat_position > POSITION_HIGH:
                self.__bottom_bat_position = POSITION_HIGH
        elif bat_direction == "LEFT":
            self.__bottom_bat_position -= POSITION_STEP
            if self.__bottom_bat_position < POSITION_LOW:
                self.__bottom_bat_position = POSITION_LOW
        else:
            pass

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # forward_speed = self.__target_twist.linear.x
        # angular_speed = self.__target_twist.angular.z

        # command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        # command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        # self.__left_motor.setVelocity(command_motor_left)
        # self.__right_motor.setVelocity(command_motor_right)
        self.__top_bat_position+=POSITION_STEP
        if self.__top_bat_position>POSITION_HIGH:
            self.__top_bat_position=POSITION_LOW
        elif self.__top_bat_position<POSITION_LOW:
            self.__top_bat_position=POSITION_LOW
        self.__top_bat_motor.setPosition(self.__top_bat_position)

        self.__bottom_bat_motor.setPosition(self.__bottom_bat_position)
        # print('bottom_bat_position:', self.__bottom_bat_position)
        if self.__bottom_rack_position:
            self.__bottom_rack.setPosition(RACK_POSITION_HIGH)
        else:
            self.__bottom_rack.setPosition(RACK_POSITION_LOW)

        # print('top_bat_position:', self.__top_bat_position)
