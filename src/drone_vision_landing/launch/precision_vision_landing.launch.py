#!/usr/bin/env python3
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_bridge_config(context, *args, **kwargs):
    model_name = LaunchConfiguration('model_name').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)

    pkg_share = get_package_share_directory('drone_vision_landing')

    if model_name == 'x500_mono_cam_down_0':
        config_file_bridge = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    else:
        raise ValueError(f'Unsupported model name: {model_name}')

    with open(config_file_bridge, 'r') as f:
        content = f.read()
    content = content.replace('${WORLD_NAME}', world_name)
    content = content.replace('${MODEL_NAME}', model_name)

    tmp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.yaml')
    tmp_file.write(content.encode())
    tmp_file.close()

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={tmp_file.name}',
        ],
        output='screen',
    )

    # --- Foxglove bridge -----------------------------------------------
    # Serves all current ROS topics over a websocket for the Foxglove Studio
    # client. Independent of model_name/world_name, so it's built here too
    # rather than needing its own OpaqueFunction — no string substitution
    # required, just static params.
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'use_sim_time': True,   # match sim clock bridged via /clock above
            'send_buffer_limit': 10000000,
        }],
        output='screen',
    )

    return [bridge_node, foxglove_bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='x500_mono_cam_down_0',
            description='Gazebo model name of the drone'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='aruco',
            description='Gazebo world name'
        ),
        OpaqueFunction(function=generate_bridge_config),

        # --- Pipeline nodes (added incrementally) ---------------------
        # ArucoDetectorNode          -> to add
        # VisualServoControllerNode -> to add
        # LandingSupervisorNode     -> to add
    ])
