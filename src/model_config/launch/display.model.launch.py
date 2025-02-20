import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions

def generate_launch_description():
    urdf_pkg_path = get_package_share_directory('model_config')
    default_urdf_path = os.path.join(urdf_pkg_path, 'urdf', 'robot.urdf')
    default_rviz_config_path = os.path.join(urdf_pkg_path, 'configs', 'robot.rviz')
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        'model',
        default_value=default_urdf_path,
        description='Absolute path to robot urdf file'
    )
    urdf_content = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )

    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}]
    )

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_pub',
        executable='joint_state_pub',
        name='joint_state_publisher',
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path],
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])

    
    
    
    
    
    
    
