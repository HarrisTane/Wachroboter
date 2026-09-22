import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =====================================================================
# Komplette Simulation fuer den Test zuhause - EIN Befehl startet alles:
#
#   /guard   : fake_odom_node + fake_scan_node + guard_robot
#   /visitor : fake_odom_node + fake_scan_node + visitor_robot
#
# Aufruf (richtiges Codewort, kein Alarm):
#   ros2 launch wachroboter_bringup sim.launch.py
#
# Aufruf (falsches Codewort, loest den Alarm aus):
#   ros2 launch wachroboter_bringup sim.launch.py codeword:=banane
#
# Mitschauen in weiteren Terminals:
#   ros2 topic echo /guard/alarm
#   ros2 topic echo /visitor/cmd_vel
# =====================================================================
def generate_launch_description():
    bringup_share = get_package_share_directory('wachroboter_bringup')
    guard_params_default = os.path.join(bringup_share, 'config', 'guard_params.yaml')
    visitor_params_default = os.path.join(bringup_share, 'config', 'visitor_params.yaml')

    # -----------------------------------------------------------------
    # Launch-Argumente
    # -----------------------------------------------------------------
    guard_params_arg = DeclareLaunchArgument(
        'guard_params_file',
        default_value=guard_params_default,
        description='YAML-Datei mit den Parametern des Wachroboters'
    )

    visitor_params_arg = DeclareLaunchArgument(
        'visitor_params_file',
        default_value=visitor_params_default,
        description='YAML-Datei mit den Parametern des Besuchers'
    )

    codeword_arg = DeclareLaunchArgument(
        'codeword',
        default_value='apfelkuchen',
        description='Codewort des Besuchers: "apfelkuchen" = richtig, sonst Alarm'
    )

    # Der Besucher wird verzoegert gestartet, damit der Service des
    # Wachroboters garantiert schon da ist. (Der Besucher wuerde zwar auch
    # von selbst warten und es erneut probieren, aber so sieht das Log
    # in der Demo aufgeraeumter aus.)
    delay_arg = DeclareLaunchArgument(
        'visitor_delay',
        default_value='4.0',
        description='Wartezeit in Sekunden, bevor der Besucher gestartet wird'
    )

    # -----------------------------------------------------------------
    # Simulierte Hardware fuer den Wachroboter, Namespace /guard
    # -----------------------------------------------------------------
    # Alle diese Nodes benutzen RELATIVE Topicnamen ("odom", "scan",
    # "cmd_vel"). Durch namespace='guard' werden daraus automatisch
    # /guard/odom, /guard/scan und /guard/cmd_vel - deshalb ist hier
    # kein einziges manuelles Remapping noetig.
    guard_fake_odom = Node(
        package='fake_robot',
        executable='fake_odom_node',
        name='fake_odom_node',
        namespace='guard',
        output='screen',
    )
    guard_fake_scan = Node(
        package='fake_robot',
        executable='fake_scan_node',
        name='fake_scan_node',
        namespace='guard',
        output='screen',
    )

    # -----------------------------------------------------------------
    # Simulierte Hardware fuer den Besucher, Namespace /visitor
    # -----------------------------------------------------------------
    visitor_fake_odom = Node(
        package='fake_robot',
        executable='fake_odom_node',
        name='fake_odom_node',
        namespace='visitor',
        output='screen',
    )
    visitor_fake_scan = Node(
        package='fake_robot',
        executable='fake_scan_node',
        name='fake_scan_node',
        namespace='visitor',
        output='screen',
    )

    # -----------------------------------------------------------------
    # Wachroboter im Namespace /guard
    # -----------------------------------------------------------------
    # KEIN name=... setzen: der Prozess enthaelt zwei Nodes
    # ("guard_robot" und "waypoint_driver"), die beide ihren eigenen
    # Namen behalten sollen. Sie landen dadurch unter
    # /guard/guard_robot und /guard/waypoint_driver.
    guard_node = Node(
        package='guard_robot',
        executable='guard_robot_node',
        namespace='guard',
        parameters=[LaunchConfiguration('guard_params_file')],
        # Topics: durch den Namespace automatisch /guard/odom, /guard/scan,
        #         /guard/cmd_vel, /guard/alarm
        # Service: soll NICHT im Namespace landen, sondern global bleiben.
        #          Deshalb hier das einzige manuelle Remapping.
        remappings=[('check_codeword', '/check_codeword')],
        output='screen',
        emulate_tty=True,
    )

    # -----------------------------------------------------------------
    # Besucher im Namespace /visitor
    # -----------------------------------------------------------------
    visitor_node = Node(
        package='visitor_robot',
        executable='visitor_robot_node',
        namespace='visitor',
        parameters=[
            LaunchConfiguration('visitor_params_file'),
            {'codeword': LaunchConfiguration('codeword')},
            # Der Client fragt den globalen Service ab. Der Parameter steht
            # zwar schon so in der YAML, hier zur Sicherheit nochmal.
            {'service_name': '/check_codeword'},
        ],
        # Topics: durch den Namespace automatisch /visitor/odom,
        #         /visitor/scan, /visitor/cmd_vel
        output='screen',
        emulate_tty=True,
    )

    # Besucher erst nach der Wartezeit starten
    visitor_delayed = TimerAction(
        period=LaunchConfiguration('visitor_delay'),
        actions=[visitor_node],
    )

    return LaunchDescription([
        guard_params_arg,
        visitor_params_arg,
        codeword_arg,
        delay_arg,
        guard_fake_odom,
        guard_fake_scan,
        visitor_fake_odom,
        visitor_fake_scan,
        guard_node,
        visitor_delayed,
    ])
