import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Pfad zur mitinstallierten Parameter-Datei
    default_params = os.path.join(
        get_package_share_directory('visitor_robot'),
        'config',
        'visitor_robot_params.yaml'
    )

    # Launch-Argument: eigene Parameter-Datei angeben koennen
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML-Datei mit den Parametern fuer visitor_robot und waypoint_driver'
    )

    # Launch-Argument: Codewort direkt beim Start umschalten
    codeword_arg = DeclareLaunchArgument(
        'codeword',
        default_value='apfelkuchen',
        description='Codewort, das an den Wachroboter geschickt wird'
    )

    # Launch-Argument: Namespace (z.B. robot2), leer = kein Namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace fuer diesen Roboter'
    )

    # Der Besucher-Node (enthaelt intern auch den waypoint_driver).
    # WICHTIG: hier bewusst KEIN name=... setzen! Der Prozess enthaelt zwei
    # Nodes ("visitor_robot" und "waypoint_driver"). Ein name=... wuerde beide
    # gleichzeitig umbenennen und die Parameter-Datei nicht mehr passen.
    visitor_node = Node(
        package='visitor_robot',
        executable='visitor_robot_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('params_file'),
            # Das Launch-Argument ueberschreibt den Wert aus der YAML-Datei
            {'codeword': LaunchConfiguration('codeword')},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        codeword_arg,
        namespace_arg,
        visitor_node,
    ])
