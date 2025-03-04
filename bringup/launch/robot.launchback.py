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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():

    rviz_config_file  = os.path.join(get_package_share_directory('ai_bot'),'rviz/robot_rviz_des.rviz')
    # RViz config file path
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_file,
                                    description='Absolute path to rviz config file')
    gui_arg = DeclareLaunchArgument(name='gui',default_value='true',choices=['true','false'],
                                    description='Flag to enable joint')
    

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'ai_bot.xacro'

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ai_bot"), "description/urdf", urdf_file_name]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}


     # URDF model path within your package
    model_arg = DeclareLaunchArgument(
        'model', default_value=urdf_file_name,
        description='Name of the URDF description to load'
    )
    #robot_description = 'robot'

    # Use built-in ROS2 URDF launch package with our own arguments
    urdf = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'ai_bot',
            'urdf_package_path': PathJoinSubstitution(['urdf', LaunchConfiguration('model')]),
            'rviz_config': LaunchConfiguration('rvizconfig')
            #'jsp_gui':LaunchConfiguration('gui')
            }.items()
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ai_bot"),
            "bringup/config",
            "diffbot_controllers.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
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
   # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    #goi cac node
    nodeslaunch = [
       #gui_arg,
        rviz_arg,
        model_arg,
        urdf, 
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,

        #joy_node,
        #teleop_node,

    ]
    
    return LaunchDescription(nodeslaunch)