import os, sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

terminal_command ="sudo usermod -aG i2c rover"
os.system(terminal_command)
terminal_command2 = "sudo chmod 666 /dev/i2c-1"
os.system(terminal_command2)
print("Permission granted")

def generate_launch_description():

    roboclaw_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'roboclaw_params.yaml'
    )
    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'osr_params.yaml'
    )

    ld = LaunchDescription()
    
    ld.add_action(
        Node(
            package='osr_control',
            executable='roboclaw_wrapper',
            name='roboclaw_wrapper',
            output='screen',
            emulate_tty=True,
            parameters=[roboclaw_params]
        )
    )
    ld.add_action(
        DeclareLaunchArgument('enable_odometry', default_value='false')
    )
    ld.add_action(
        Node(
            package='osr_control',
            executable='servo_control',
            name='servo_wrapper',
            output='screen',
            emulate_tty=True,
            parameters=[{'centered_pulse_widths': [0, 0, 0, 0]}]  # pulse width where the corner motors are in their default position, see rover_bringup.md.
        ) # 165, 134, 145, 160
    )
    ld.add_action(
        DeclareLaunchArgument('enable_odometry', default_value='false')
    )
    ld.add_action(
        Node(
            package='osr_control',
            executable='rover',
            name='rover',
            output='screen',
            emulate_tty=True,
            parameters=[osr_params,
                        {'enable_odometry': LaunchConfiguration('enable_odometry')}]
        )
    )
    ld.add_action(
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            emulate_tty=True,
            parameters=[
                # {"scale_linear.x": 0.4},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                {"scale_linear.x": 0.02},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor    {"scale_linear": 0.05},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor

                # {"axis_linear.x": 4},
                {"axis_linear.x": 1},
                # {"axis_angular.yaw": 0},  # which joystick axis to use for driving
                {"axis_angular.yaw": 0},  # which joystick axis to use for driving
                # {"scale_angular.yaw": 1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"axis_angular.pitch": 2},  # axis to use for in-place rotation
                {
                    "scale_angular.yaw": 1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                # {"scale_angular.pitch": 0.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"scale_angular_turbo.yaw": 3.95},  # scale to apply to angular speed, in rad/s: scale_linear_turbo / min_radius
                {"scale_linear_turbo.x": 1.78},  # scale to apply to linear speed, in m/s
                # {"enable_button": 4},  # which button to press to enable movement
                {"require_enable_button": True},
                # {"enable_button": 0},  # which button to press to enable movement
                {"enable_turbo_button": 0}  # -1 to disable turbo
                # {"enable_turbo_button": -1}  # -1 to disable turbo
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel_intuitive')
            ]
        )
    )
    ld.add_action(
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"autorepeat_rate": 10.0},
                {"device_id": 0},  # This might be different on your computer. Run `ls -l /dev/input/event*`. If you have event1, put 1.
            ]        
        )
    )

    return ld


