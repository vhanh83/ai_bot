# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import launch
import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # Get URDF via xacro

    package_name='ai_bot' #<--- CHANGE ME
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "urdf", "ai_bot.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", "ai_bot.rviz"]
    )

    #joy node
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                     get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
     )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    #laser Hokuyo
    urg_node = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([FindPackageShare('urg_node2'), '/launch', '/urg_node2.launch.py']),
        #launch_arguments={
            
            #'auto_start': LaunchConfiguration('true')
            
           # }.items()
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joystick,
        twist_mux,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        urg_node,
        
    ]

    return LaunchDescription(nodes)
# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import launch
import yaml
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # Get URDF via xacro

    package_name='ai_bot' #<--- CHANGE ME
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "urdf", "ai_bot.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", "map_maker_config.rviz"]
    )

    #joy node
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                     get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
     )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    #laser Hokuyo
    urg_node = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([FindPackageShare('urg_node2'), '/launch', '/urg_node2.launch.py']),
        #launch_arguments={
            
            #'auto_start': LaunchConfiguration('true')
            
           # }.items()
    )
    #slam_toolbox online_async_launch.py params_file:=~/anh_ws/src/ai_bot/bringup/config/mapper_params_online_async.yaml use_sim_time:= true
    #map
    makemap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ai_bot'), '/launch', '/online_async_launch.py']),
       
        
        #parameters=["params_file", map_params],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joystick,
        twist_mux,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        urg_node,
        makemap
    ]

    return LaunchDescription(nodes)
