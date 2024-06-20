import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    
    # launch_dir = os.path.join(get_package_share_directory('rosbridge_server'),'launch')
    
    return LaunchDescription([
        # Run node
        # Rosbrige lan:
        Node(
            package='rosbridge_server',
            namespace='',
            executable='rosbridge_websocket',
            name='rosbridge_websocket_lan',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'port': 9090,
                    'address': '192.168.1.10',
                    'retry_startup_delay': 5.0,
                    'fragment_timeout': 600,
                    'delay_between_messages': 0,
                    'max_message_size': 10000000,
                    'unregister_timeout': 10.0,
                    'use_compression': False,
                    'call_services_in_new_thread': False,
                    'send_action_goals_in_new_thread': False,
                    'topics_glob': '',
                    'services_glob': '',
                    'params_glob': '',
                    'bson_only_mode': False,
                }
            ]
        ),

        Node(
            package='rosapi',
            namespace='',
            executable='rosapi_node',
            name='rosapi_lan',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'topics_glob': '',
                    'services_glob': '',
                    'params_glob': '',
                }
            ]
        ),

        # Rosbrige wlan:
        Node(
            package='rosbridge_server',
            namespace='',
            executable='rosbridge_websocket',
            name='rosbridge_websocket_wlan',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'port': 9090,
                    'address': '192.168.48.245',
                    'retry_startup_delay': 5.0,
                    'fragment_timeout': 600,
                    'delay_between_messages': 0,
                    'max_message_size': 10000000,
                    'unregister_timeout': 10.0,
                    'use_compression': False,
                    'call_services_in_new_thread': False,
                    'send_action_goals_in_new_thread': False,
                    'topics_glob': '',
                    'services_glob': '',
                    'params_glob': '',
                    'bson_only_mode': False,
                }
            ]
        ),

        Node(
            package='rosapi',
            namespace='',
            executable='rosapi_node',
            name='rosapi_wlan',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'topics_glob': '',
                    'services_glob': '',
                    'params_glob': '',
                }
            ]
        ),

        # Node control service data:
        Node(
            package='vdm_lk2_machine',
            namespace='',
            executable='pc_service_ros',
            name='PC_service_ros',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'password': '10448'
                }
            ]
        ),

        Node(
            package='vdm_lk2_machine',
            namespace='',
            executable='plc_FX_service_ros',
            name='PLC_FX3U_ros_thanhhinh',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'PLC_model': 'FX3U_ThanhHinh',
                    'PLC_IP_address': '192.168.1.250',
                    'PLC_Port_address': 8000,
                    'maximum_dates': 365,
                    'password': '10448'
                }
            ]
        ),

        Node(
            package='vdm_lk2_machine',
            namespace='',
            executable='plc_FX_service_ros',
            name='PLC_FX5U_ros_thanhhinh',
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    'PLC_model': 'FX5U_ThanhHinh',
                    'PLC_IP_address': '192.168.1.251',
                    'PLC_Port_address': 8000,
                    'maximum_dates': 365,
                    'password': '10448'
                }
            ]
        ),
    ])
