#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    show_rviz = LaunchConfiguration('show_rviz', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    moveit_config_dump = LaunchConfiguration('moveit_config_dump')
    rviz_config = LaunchConfiguration('rviz_config', default='')
    octomap_enable = LaunchConfiguration('octomap_enable', default=False)
    octomap_resolution = LaunchConfiguration('octomap_resolution', default=0.02)
    octomap_frame = LaunchConfiguration('octomap_frame', default='link_base')

    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader)
    moveit_config_package_name = 'xarm_moveit_config'

    move_group_parameters = [
        moveit_config_dict,
        {'use_sim_time': use_sim_time},
    ]

    if octomap_enable.perform(context) in ('True', 'true', '1'):
        sensor_manager_parameters = {
            'moveit_sensor_manager': 'moveit_ros_perception/PointCloudOctomapUpdater',
            'sensors': ['right_cam', 'left_cam'],
            'octomap_frame': octomap_frame.perform(context),
            'octomap_resolution': float(octomap_resolution.perform(context)),
            # 'max_range': 2.0,

            'right_cam.sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater',
            'right_cam.point_cloud_topic': '/camera/right/depth/color/points',
            'right_cam.max_range': 2.0,
            'right_cam.point_subsample': 2,
            # 'right_cam.sensor_plugin': 'occupancy_map_monitor/DepthImageOctomapUpdater',
            # 'right_cam.image_topic': '/camera/right/aligned_depth_to_color/image_raw',
            # 'right_cam.camera_info_topic': '/camera/right/aligned_depth_to_color/camera_info',
            # 'right_cam.queue_size': 5,
            # 'right_cam.near_clipping_plane_distance': 0.2,
            # 'right_cam.far_clipping_plane_distance': 3.0,
            # 'right_cam.shadow_threshold': 0.2,
            'right_cam.padding_offset': 0.15,
            'right_cam.padding_scale': 1.0,
            'right_cam.max_update_rate': 5.0,
            'right_cam.filtered_cloud_topic': '/right_filtered_cloud',

            'left_cam.sensor_plugin': 'occupancy_map_monitor/PointCloudOctomapUpdater',
            'left_cam.point_cloud_topic': '/camera/left/depth/color/points',
            'left_cam.max_range': 2.0,
            'left_cam.point_subsample': 2,
            # 'left_cam.sensor_plugin': 'occupancy_map_monitor/DepthImageOctomapUpdater',
            # 'left_cam.image_topic': '/camera/left/aligned_depth_to_color/image_raw',
            # 'left_cam.camera_info_topic': '/camera/left/aligned_depth_to_color/camera_info',
            # 'left_cam.queue_size': 5,
            # 'left_cam.near_clipping_plane_distance': 0.2,
            # 'left_cam.far_clipping_plane_distance': 3.0,
            # 'left_cam.shadow_threshold': 0.2,
            'left_cam.padding_offset': 0.15,
            'left_cam.padding_scale': 1.0,
            'left_cam.max_update_rate': 5.0,
            'left_cam.filtered_cloud_topic': '/left_filtered_cloud',
        }
        move_group_parameters.append(sensor_manager_parameters)

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_parameters,
    )

    # rviz with moveit configuration
    if not rviz_config.perform(context):
        rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    else:
        rviz_config_file = rviz_config
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config_dict['robot_description'],
                'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
                'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
                'robot_description_planning': moveit_config_dict['robot_description_planning'],
                'planning_pipelines': moveit_config_dict['planning_pipelines'],
                'use_sim_time': use_sim_time
            }
        ],
        condition=IfCondition(show_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    xyz = attach_xyz.perform(context)[1:-1].split(' ')
    rpy = attach_rpy.perform(context)[1:-1].split(' ')
    args = xyz + rpy + [attach_to.perform(context), '{}link_base'.format(prefix.perform(context))]

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=args,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    robot_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_robot_planner.launch.py'])),
        condition=IfCondition(no_gui_ctrl),
        launch_arguments={
            'moveit_config_dump': moveit_config_dump,
        }.items(),
    )

    return [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=rviz2_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        rviz2_node,
        static_tf,
        move_group_node,
        robot_planner_node_launch
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
