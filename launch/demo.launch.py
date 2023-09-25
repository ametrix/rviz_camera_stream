import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def launch_setup(context, *args, **kwargs):

    rviz_config_package_name = LaunchConfiguration("rviz_config_package_name", default="rviz_camera_stream")
    rviz_config_file_name = LaunchConfiguration("rviz_config_file_name", default="rviz_camera_stream")

    rviz_config = os.path.join(
        get_package_share_directory(rviz_config_package_name.perform(context)),
        "config",
        rviz_config_file_name.perform(context) + ".rviz",
    )
    print("Loading rviz config from file: ", rviz_config)

    nodes_to_start = []

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        emulate_tty=True,
        arguments=["-d", rviz_config],
        on_exit=Shutdown(),
        condition=launch.conditions.IfCondition(LaunchConfiguration("launch_rviz")),
    )

    camera1_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera1_transform",
        output="screen",
        emulate_tty=True,
        arguments=["5.5", "1.5", "-5.5", "0", "0", "0", "1", "map", "camera1"],
        on_exit=Shutdown(),
    )

    camera2_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera2_transform",
        output="screen",
        emulate_tty=True,
        arguments=["5.5", "0.5", "-0.5", "0", "0", "0", "1", "map", "camera2"],
        on_exit=Shutdown(),
    )

    pub_extra_options = ['--qos-reliability', 'reliable',
                         '--qos-durability', 'transient_local']


    camera1_config_action = ExecuteProcess(
        cmd=(['ros2', 'topic', 'pub'] + pub_extra_options +
             ['/rviz/camera1/info', 'sensor_msgs/CameraInfo',
              '''{header: {stamp: 'now', frame_id: 'camera1'},
                height: 480, width: 640, distortion_model: 'plumb_bob',
                d: [0.0],
                k: [300.0, 0.0, 640, 0.0, 300.0, 360.0, 0.0, 0.0, 1.0],
                r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                p: [300.0, 0.0, 640, 0.0, 0.0, 300.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                binning_x: 0, binning_y: 0,
                roi: {x_offset: 0, y_offset: 0, height: 480, width: 640, do_rectify: false}}''', '-1'
             ]),
        additional_env={'PYTHONUNBUFFERED': '1'}, output='screen'
    )

    camera1_config_event_hanadler = RegisterEventHandler(
        OnProcessStart(
            target_action=camera1_transform_node,
            on_start=[
                LogInfo(msg='RViz started, applying camera1 config...'),
                camera1_config_action
            ]
        )
    )

    camera2_config_action = ExecuteProcess(
        cmd=(['ros2', 'topic', 'pub'] + pub_extra_options +
             ['/rviz/camera2/info', 'sensor_msgs/CameraInfo',
              '''{header: {stamp: 'now', frame_id: 'camera2'},
                height: 480, width: 640, distortion_model: 'plumb_bob',
                d: [0.0],
                k: [300.0, 0.0, 640, 0.0, 300.0, 360.0, 0.0, 0.0, 1.0],
                r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                p: [300.0, 0.0, 640, 0.0, 0.0, 300.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                binning_x: 0, binning_y: 0,
                roi: {x_offset: 0, y_offset: 0, height: 480, width: 640, do_rectify: false}}''', '-1'
             ]),
        additional_env={'PYTHONUNBUFFERED': '1'}, output='screen'
    )

    camera2_config_event_hanadler = RegisterEventHandler(
        OnProcessStart(
            target_action=camera1_transform_node,
            on_start=[
                LogInfo(msg='RViz started, applying camera2 config...'),
                camera2_config_action
            ]
        )
    )

    nodes_to_start.append(rviz_node)
    nodes_to_start.append(camera1_transform_node)
    nodes_to_start.append(camera2_transform_node)
    nodes_to_start.append(camera1_config_event_hanadler)
    nodes_to_start.append(camera2_config_event_hanadler)
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            name="launch_rviz",
            default_value="True",
            description="Enable/disable spawning of RViz node.",
        )
    )

    return launch.LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
