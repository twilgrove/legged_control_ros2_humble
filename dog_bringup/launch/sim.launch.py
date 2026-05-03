import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import ExecuteProcess


def generate_launch_description():

    gazebo_pkg = get_package_share_directory("gazebo_ros")
    pkg = get_package_share_directory("dog_bringup")

    xacro_file = os.path.join(pkg, "xacro", "cdut_dog", "dog.xacro")
    is_sim = "true"
    xacro_command = ["xacro ", xacro_file, " is_sim:=", is_sim]
    urdf_output_dir = os.path.join(pkg, "config", "cdut_dog", "description")
    urdf_output_file = os.path.join(urdf_output_dir, "dog.urdf")
    world_file = os.path.join(pkg, "config", "dog.world")
    rviz__config_file = os.path.join(pkg, "dog.rviz")

    generate_urdf = ExecuteProcess(
        cmd=[
            f"mkdir -p {urdf_output_dir} && "
            f"xacro {xacro_file} is_sim:={is_sim} -o {urdf_output_file}"
        ],
        shell=True,
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz__config_file],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments=[("world", world_file), ("verbose", "true")],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "dog_robot",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.04",
            "-timeout",
            "300",
        ],
        output="screen",
    )

    controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["DogNmpcWbcControllerSim"],
    )

    dog_controller_container = ComposableNodeContainer(
        name="dog_controller_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="dog_controllers",
                plugin="dog_controllers::TargetTrajectoriesPublisher",
                name="target_trajectories_publisher",
            ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(xacro_command), value_type=str
                        )
                    }
                ],
            ),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ld = LaunchDescription()

    ld.add_action(generate_urdf)
    ld.add_action(rviz_node)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(joint_state_broadcaster_spawner)

    ld.add_action(controller)
    ld.add_action(dog_controller_container)

    return ld
