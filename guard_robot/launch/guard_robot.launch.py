import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Pfad zur mitinstallierten Parameter-Datei
    default_params = os.path.join(
        get_package_share_directory('guard_robot'),
        'config',
        'guard_robot_params.yaml'
    )

    # Launch-Argument: eigene Parameter-Datei angeben koennen
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML-Datei mit den Parametern fuer guard_robot und waypoint_driver'
    )

    # Launch-Argument: Namespace (z.B. robot1), leer = kein Namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace fuer diesen Roboter (leer lassen fuer /check_codeword)'
    )

    # Der Wachroboter-Node (enthaelt intern auch den waypoint_driver).
    # WICHTIG: hier bewusst KEIN name=... setzen! Der Prozess enthaelt zwei
    # Nodes ("guard_robot" und "waypoint_driver"). Ein name=... wuerde beide
    # gleichzeitig umbenennen und die Parameter-Datei nicht mehr passen.
    guard_node = Node(
        package='guard_robot',
        executable='guard_robot_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        namespace_arg,
        guard_node,
    ])
