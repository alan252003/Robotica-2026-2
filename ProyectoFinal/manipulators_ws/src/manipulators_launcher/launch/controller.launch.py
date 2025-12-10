from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
  description_path = get_package_share_directory("manipulators_description")
  model_path = os.path.join(description_path, "urdf", "robot_manipulator.urdf")
  rviz_conf_path = os.path.join(description_path, "rviz", "rviz_config.rviz")
  robot_description = {"robot_description": Command(["xacro ", model_path])}

# Para primer manipulador
  controller_manager_xy_node = Node(
    package='manipulators_control',
    executable="controller_manager_xy",
    name="controller_manager_xy"
  )

  manipulator_controller_xy_node = Node(
    package='manipulators_control',
    executable="manipulator_controller_xy",
    name="manipulator_controller_xy"
  )

  hardware_interface_xy_node = Node(
    package='manipulators_control',
    executable="hardware_interface_xy",
    name="hardware_interface_xy"
  )

# Para segundo manipulador
  controller_manager_zy_node = Node(
    package='manipulators_control',
    executable="controller_manager_zy",
    name="controller_manager_zy"
  )

  manipulator_controller_zy_node = Node(
    package='manipulators_control',
    executable="manipulator_controller_zy",
    name="manipulator_controller_zy"
  )

  hardware_interface_zy_node = Node(
    package='manipulators_control',
    executable="hardware_interface_zy",
    name="hardware_interface_zy"
  )


  rviz_node = Node(
    package='rviz2',
    executable="rviz2",
    arguments=["-d", rviz_conf_path]
  )
  jsp_node = Node(
    package='joint_state_publisher_gui',
    executable="joint_state_publisher_gui"
  )
  rsp_node = Node(
    package='robot_state_publisher',
    executable="robot_state_publisher",
    parameters=[robot_description]
  )
  return LaunchDescription([
    controller_manager_xy_node,
    manipulator_controller_xy_node,
    hardware_interface_xy_node,
    controller_manager_zy_node,
    manipulator_controller_zy_node,
    hardware_interface_zy_node,
    rviz_node, 
    rsp_node
    ])
