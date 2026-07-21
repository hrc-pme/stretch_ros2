from ament_index_python.packages import get_package_share_path
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.descriptions
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():
    stretch_core_path = get_package_share_path('stretch_core')

    declare_tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='stretch3',
        description='Prefix for TF frame IDs'
    )

    declare_broadcast_odom_tf_arg = DeclareLaunchArgument(
        'broadcast_odom_tf',
        default_value='False', choices=['True', 'False'],
        description='Whether to broadcast the odom TF'
    )

    declare_fail_out_of_range_goal_arg = DeclareLaunchArgument(
        'fail_out_of_range_goal',
        default_value='False', choices=['True', 'False'],
        description='Whether the motion action servers fail on out-of-range commands'
    )

    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='position', choices=['position', 'navigation', 'trajectory', 'gamepad'],
        description='The mode in which the ROS driver commands the robot'
    )

    declare_controller_arg = DeclareLaunchArgument(
        'calibrated_controller_yaml_file',
        default_value=str(stretch_core_path / 'config' / 'controller_calibration_head.yaml'),
        description='Path to the calibrated controller args file'
    )

    stretch_description_urdf_dir = Path(get_package_share_path('stretch_description')) / 'urdf'
    package_description_xacro = stretch_description_urdf_dir / 'stretch.urdf'
    exported_robot_urdf = Path('/root/stretch_user/stretch-se3-3092/exported_urdf/stretch.urdf')

    if package_description_xacro.exists():
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
            Command(['xacro ', str(package_description_xacro)]), value_type=str
        )
    elif exported_robot_urdf.exists():
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
            Command(['cat ', str(exported_robot_urdf)]), value_type=str
        )
    else:
        # Last-resort fallback for environments where only partial description assets are installed.
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
            Command(['xacro ', str(stretch_description_urdf_dir / 'stretch_base_imu.xacro')]), value_type=str
        )

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 output='log',
                                 parameters=[{'source_list': ['/stretch/joint_states']},
                                             {'rate': 30.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content},
                                             {'publish_frequency': 30.0},
                                             {'frame_prefix': [LaunchConfiguration('tf_prefix'), '/']}],
                                 arguments=['--ros-args', '--log-level', 'error'],)

    stretch_driver_params = [
        {'rate': 30.0,
         'timeout': 0.5,
         'controller_calibration_file': LaunchConfiguration('calibrated_controller_yaml_file'),
         'broadcast_odom_tf': LaunchConfiguration('broadcast_odom_tf'),
         'fail_out_of_range_goal': LaunchConfiguration('fail_out_of_range_goal'),
            'mode': LaunchConfiguration('mode'),
            'tf_frame_prefix': LaunchConfiguration('tf_prefix')}
    ]

    stretch_driver = Node(package='stretch_core',
                          executable='stretch_driver',
                          emulate_tty=True,
                          output='screen',
                          remappings=[('cmd_vel', '/stretch/cmd_vel'),
                                      ('joint_states', '/stretch/joint_states'),
                                      ('odom', '/stretch3/odom'),
                                      ('battery', '/stretch3/battery'),
                                      ('is_homed', '/stretch3/is_homed'),
                                      ('mode', '/stretch3/mode'),
                                      ('tool', '/stretch3/tool'),
                                      ('is_streaming_position', '/stretch3/is_streaming_position'),
                                      ('imu_mobile_base', '/stretch3/imu_mobile_base'),
                                      ('magnetometer_mobile_base', '/stretch3/magnetometer_mobile_base'),
                                      ('imu_wrist', '/stretch3/imu_wrist'),
                                      ('is_runstopped', '/stretch3/is_runstopped'),
                                      ('is_gamepad_dongle', '/stretch3/is_gamepad_dongle'),
                                      ('stretch_gamepad_state', '/stretch3/stretch_gamepad_state'),
                                      ('joint_limits', '/stretch3/joint_limits')],
                          parameters=stretch_driver_params)

    return LaunchDescription([declare_tf_prefix_arg,
                              declare_broadcast_odom_tf_arg,
                              declare_fail_out_of_range_goal_arg,
                              declare_mode_arg,
                              declare_controller_arg,
                              joint_state_publisher,
                              robot_state_publisher,
                              stretch_driver])
