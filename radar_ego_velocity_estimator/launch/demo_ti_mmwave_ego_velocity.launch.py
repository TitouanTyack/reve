# This file is part of REVE - Radar Ego Velocity Estimator 
# Developped by Christopher Doer <christopher.doer@kit.edu>
# Author of this file: Titouan Tyack <titouan.tyack@isae-supaero.fr>
from ast import arguments
from http.server import executable
import os

from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('radar_ego_velocity_estimator'),
        'config',
        'params_demo_dataset.yaml'
        )
    
    radar_ego_velocity_estimator = Node(
            package='radar_ego_velocity_estimator',
            executable='radar_ego_velocity_estimation_ros_node',
            name='radar_ego_velocity_estimation_ros_node',
            parameters=[config],
            remappings = [
                ("radar/scan",      "/ti_mmwave/radar_scan_pcl"),
                ("radar/trigger" ,  "/sensor_platform/radar_right/trigger"),
            ],
            output='screen',
        )
    
    # velocity_estimation_evaluator = Node(
    #         package='radar_ego_velocity_estimator',
    #         executable='velocity_estimation_evaluator',
    #         name='velocity_estimation_evaluator',
    #         parameters=[config],
    #         remappings = [
    #             ("radar/scan",      "/ti_mmwave/radar_scan_pcl"),
    #             ("radar/trigger" ,  "/sensor_platform/radar_right/trigger"),
    #         ],
    #         output='screen',
    #     )

    return LaunchDescription([GroupAction(
     actions=[
         radar_ego_velocity_estimator,
    ])])

