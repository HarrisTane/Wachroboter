import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =====================================================================
# Startet NUR den Wachroboter - fuer den Einsatz auf dem echten Roboter.
# Alle Parameter kommen aus config/guard_params.yaml.
#
# Aufruf:
#   ros2 launch wachroboter_bringup guard.launch.py
#
# Mit eigener Parameter-Datei:
#   ros2 launch wachroboter_bringup guard.launch.py params_file:=/pfad/zu/meine.yaml
# =====================================================================
def generate_launch_description():
    # Pfad zur mitinstallierten Standard-Parameterdatei ermitteln
    default_params = os.path.join(
        get_package_share_directory('wachroboter_bringup'),
        'config',
        'guard_params.yaml'
    )

    # --- Launch-Argumente ---

    # Welche YAML-Datei soll benutzt werden?
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML-Datei mit den Parametern fuer guard_robot und waypoint_driver'
    )

    # Namespace: leer lassen fuer den echten Roboter (Topics heissen dann
    # /odom, /cmd_vel, /scan, /alarm und der Service /check_codeword)
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace fuer diesen Roboter (leer = keine Namespaces)'
    )

    # --- Der Node ---
    # WICHTIG: hier bewusst KEIN name=... setzen! Der Prozess enthaelt zwei
    # Nodes ("guard_robot" und "waypoint_driver"). Ein name=... wuerde beide
    # gleichzeitig umbenennen.
    guard_node = Node(
        package='guard_robot',
        executable='guard_robot_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        # Der Service muss immer global erreichbar sein, auch wenn der Node
        # in einem Namespace laeuft -> auf den absoluten Namen umbiegen.
        remappings=[('check_codeword', '/check_codeword')],
        output='screen',
        emulate_tty=True,   # sorgt fuer farbige Log-Ausgaben
    )

    return LaunchDescription([
        params_arg,
        namespace_arg,
        guard_node,
    ])
