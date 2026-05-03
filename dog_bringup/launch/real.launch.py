import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("dog_bringup")

    xacro_file = os.path.join(pkg, "xacro", "cdut_dog", "dog.xacro")
    is_sim = "false"
    xacro_command = ["xacro ", xacro_file, " is_sim:=", is_sim]
    urdf_output_dir = os.path.join(pkg, "config", "cdut_dog", "description")
    urdf_output_file = os.path.join(urdf_output_dir, "dog.urdf")

    controller_config = os.path.join(
        pkg, "config", "cdut_dog", "robot_controllers.yaml"
    )

    generate_urdf = ExecuteProcess(
        cmd=[
            f"mkdir -p {urdf_output_dir} && "
            f"xacro {xacro_file} is_sim:={is_sim} -o {urdf_output_file}"
        ],
        shell=True,
        output="screen",
    )

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_command), value_type=str)
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        emulate_tty=True,
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    dog_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["DogNmpcWbcController"],
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
        ],
        output="screen",
    )

    rviz_config_file = os.path.join(pkg, "dog.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(generate_urdf)
    ld.add_action(ros2_control_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(imu_broadcaster_spawner)
    ld.add_action(dog_controller_spawner)

    # ld.add_action(dog_controller_container)
    # ld.add_action(rviz_node)

    return ld
