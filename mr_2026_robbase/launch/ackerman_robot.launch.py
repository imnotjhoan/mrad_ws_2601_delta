import os #tomar info de sistema operativo

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription 
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import xacro #hace lectura de informaxión xacro de robot

pkg_folder = 'mr_2026_robbase' #nombre de paquete
robot_file = 'ackerman_robot.urdf.xacro'  #nombre de archivo donde está el modelo del robot

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    default_model_path = os.path.join(pkg_path + '/model/' + robot_file)

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')

    # Process the URDF file
    robot_description_config = xacro.process_file(default_model_path) #convertimos xacro en urdf

    params = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher_node = Node( #estado robot
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params] #lee parametros de creación de robot
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node( #estado articulaciones
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Launch!
    return LaunchDescription([
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
    ])