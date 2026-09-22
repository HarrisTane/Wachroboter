import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =====================================================================
# Startet NUR den Besucher-Roboter - fuer den Einsatz auf dem echten Roboter.
# Alle Parameter kommen aus config/visitor_params.yaml.
#
# Aufruf mit richtigem Codewort:
#   ros2 launch wachroboter_bringup visitor.launch.py
#
# Aufruf mit falschem Codewort (loest den Alarm aus):
#   ros2 launch wachroboter_bringup visitor.launch.py codeword:=banane
# =====================================================================
def generate_launch_description():
    # Pfad zur mitinstallierten Standard-Parameterdatei ermitteln
    default_params = os.path.join(
        get_package_share_directory('wachroboter_bringup'),
        'config',
        'visitor_params.yaml'
    )

    # --- Launch-Argumente ---

    # Welche YAML-Datei soll benutzt werden?
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML-Datei mit den Parametern fuer visitor_robot und waypoint_driver'
    )

    # Codewort direkt beim Start umschalten, ohne die YAML anzufassen.
    # Praktisch fuer die Demo: einmal richtig, einmal falsch.
    codeword_arg = DeclareLaunchArgument(
        'codeword',
        default_value='apfelkuchen',
        description='Codewort, das an den Wachroboter geschickt wird'
    )

    # Namespace: leer lassen fuer den echten Roboter
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace fuer diesen Roboter (leer = keine Namespaces)'
    )

    # --- Der Node ---
    # WICHTIG: hier bewusst KEIN name=... setzen! Der Prozess enthaelt zwei
    # Nodes ("visitor_robot" und "waypoint_driver").
    visitor_node = Node(
        package='visitor_robot',
        executable='visitor_robot_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            # 1. die YAML-Datei
            LaunchConfiguration('params_file'),
            # 2. das Launch-Argument ueberschreibt den Wert aus der YAML
            {'codeword': LaunchConfiguration('codeword')},
        ],
        output='screen',
        emulate_tty=True,   # sorgt fuer farbige Log-Ausgaben
    )

    return LaunchDescription([
        params_arg,
        codeword_arg,
        namespace_arg,
        visitor_node,
    ])
