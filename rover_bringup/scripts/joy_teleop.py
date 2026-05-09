#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rover_msgs.srv import SetLEDBrightness
import pygame


def get_joystick_names():
    """Detect connected joysticks and return their names"""
    pygame.init()
    pygame.joystick.init()

    joystick_names = []
    joystick_count = pygame.joystick.get_count()

    if joystick_count == 0:
        print("No joystick detected")
    else:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            joystick_names.append(joystick.get_name())

    pygame.quit()
    return joystick_names


class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)

        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('deadzone', 0.2)
        self.declare_parameter('linear_gear_default', 1.0)
        self.declare_parameter('angular_gear_default', 1.0)

        # Joystick button/axis mapping parameters
        self.declare_parameter('linear_gear_button', 13)
        self.declare_parameter('angular_gear_button', 14)
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 2)

        # Headlight parameters
        self.declare_parameter('headlight_up_button', 1)
        self.declare_parameter('headlight_down_button', 2)
        self.declare_parameter('headlight_step', 64.0)

        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.speed_scale = self.get_parameter('speed_scale').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.linear_gear = self.get_parameter('linear_gear_default').get_parameter_value().double_value
        self.angular_gear = self.get_parameter('angular_gear_default').get_parameter_value().double_value

        # Joystick button/axis mappings from params
        self.linear_gear_button = self.get_parameter('linear_gear_button').get_parameter_value().integer_value
        self.angular_gear_button = self.get_parameter('angular_gear_button').get_parameter_value().integer_value
        self.linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value

        # Headlight params
        self.headlight_up_btn = self.get_parameter('headlight_up_button').get_parameter_value().integer_value
        self.headlight_down_btn = self.get_parameter('headlight_down_button').get_parameter_value().integer_value
        self.headlight_step = self.get_parameter('headlight_step').get_parameter_value().double_value

        # Log loaded parameters
        self.get_logger().info('Joy Teleop Parameters:')
        self.get_logger().info(f'  Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'  Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'  Speed scale: {self.speed_scale}')
        self.get_logger().info(f'  Deadzone: {self.deadzone}')
        self.get_logger().info(f'  Linear gear: {self.linear_gear}')
        self.get_logger().info(f'  Angular gear: {self.angular_gear}')
        self.get_logger().info(f'  Linear gear button: {self.linear_gear_button}')
        self.get_logger().info(f'  Angular gear button: {self.angular_gear_button}')
        self.get_logger().info(f'  Linear axis: {self.linear_axis}')
        self.get_logger().info(f'  Angular axis: {self.angular_axis}')
        self.get_logger().info(f'  Headlight up button: {self.headlight_up_btn}')
        self.get_logger().info(f'  Headlight down button: {self.headlight_down_btn}')
        self.get_logger().info(f'  Headlight step: {self.headlight_step}')

        # Create publisher
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber
        self.sub_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Headlight service client
        self.headlight_client = self.create_client(SetLEDBrightness, '/ugv/set_headlights')
        self.headlight_brightness = 0.0  # Start off

        # Detect joystick
        joysticks = get_joystick_names()
        self.joystick = joysticks[0] if len(joysticks) != 0 else None

        if self.joystick is None:
            self.get_logger().error("No joystick detected! Please connect a controller.")
        else:
            self.get_logger().info(f"Detected joystick: {self.joystick}")

        # Button press state tracking
        self.last_linear_button_state = 0
        self.last_angular_button_state = 0
        self.last_light_up_state = 0
        self.last_light_down_state = 0

        self.get_logger().info("Joy teleop node started - Ready for input!")

    def set_headlights(self, brightness):
        """Call the headlight service asynchronously"""
        if not self.headlight_client.service_is_ready():
            self.get_logger().warn("Headlight service not available")
            return
        req = SetLEDBrightness.Request()
        req.brightness = brightness
        self.headlight_client.call_async(req)
        self.get_logger().info(f"Headlights: {brightness:.0f}/255 ({brightness / 255 * 100:.0f}%)")

    def joy_callback(self, joy_data):
        """Process joystick input and publish cmd_vel"""
        if not isinstance(joy_data, Joy):
            return

        # --- Headlight brightness up ---
        light_up_pressed = joy_data.buttons[self.headlight_up_btn] == 1
        if light_up_pressed and self.last_light_up_state == 0:
            self.headlight_brightness = min(255.0, self.headlight_brightness + self.headlight_step)
            self.set_headlights(self.headlight_brightness)
        self.last_light_up_state = joy_data.buttons[self.headlight_up_btn]

        # --- Headlight brightness down ---
        light_down_pressed = joy_data.buttons[self.headlight_down_btn] == 1
        if light_down_pressed and self.last_light_down_state == 0:
            self.headlight_brightness = max(0.0, self.headlight_brightness - self.headlight_step)
            self.set_headlights(self.headlight_brightness)
        self.last_light_down_state = joy_data.buttons[self.headlight_down_btn]

        # --- Linear gear control ---
        linear_button_pressed = joy_data.buttons[self.linear_gear_button] == 1
        if linear_button_pressed and self.last_linear_button_state == 0:
            if self.linear_gear == 1.0:
                self.linear_gear = 1.0 / 3
            elif self.linear_gear == 1.0 / 3:
                self.linear_gear = 2.0 / 3
            elif self.linear_gear == 2.0 / 3:
                self.linear_gear = 1.0
            self.get_logger().info(f"Linear gear: {self.linear_gear:.2f}")
        self.last_linear_button_state = joy_data.buttons[self.linear_gear_button]

        # --- Angular gear control ---
        angular_button_pressed = joy_data.buttons[self.angular_gear_button] == 1
        if angular_button_pressed and self.last_angular_button_state == 0:
            if self.angular_gear == 1.0:
                self.angular_gear = 1.0 / 4
            elif self.angular_gear == 1.0 / 4:
                self.angular_gear = 1.0 / 2
            elif self.angular_gear == 1.0 / 2:
                self.angular_gear = 3.0 / 4
            elif self.angular_gear == 3.0 / 4:
                self.angular_gear = 1.0
            self.get_logger().info(f"Angular gear: {self.angular_gear:.2f}")
        self.last_angular_button_state = joy_data.buttons[self.angular_gear_button]

        # --- Calculate and publish cmd_vel ---
        linear_axis_value = self.filter_deadzone(joy_data.axes[self.linear_axis])
        angular_axis_value = self.filter_deadzone(joy_data.axes[self.angular_axis])

        linear_speed = linear_axis_value * self.max_linear_speed * self.linear_gear * self.speed_scale
        angular_speed = angular_axis_value * self.max_angular_speed * self.angular_gear * self.speed_scale

        linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        twist = Twist()
        twist.linear.x = linear_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed

        self.pub_cmd_vel.publish(twist)

    def filter_deadzone(self, value):
        """Filter out small joystick values to prevent drift"""
        if abs(value) < self.deadzone:
            return 0.0
        return value


def main(args=None):
    rclpy.init(args=args)
    joy_ctrl = JoyTeleop('joy_teleop')

    try:
        rclpy.spin(joy_ctrl)
    except KeyboardInterrupt:
        pass
    finally:
        joy_ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()