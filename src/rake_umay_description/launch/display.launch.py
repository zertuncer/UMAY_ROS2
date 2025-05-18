import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros
import os

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='rake_umay_description').find('rake_umay_description')
    urdfModelPath = os.path.join(pkgPath, 'urdf/rake_umay_v2.urdf.xacro')
    rvizConfigPath = os.path.join(pkgPath, 'rviz/config.rviz')

    print(urdfModelPath)
    # xacro'u çalıştırıp XML çıktısını alıyoruz ve string olarak işaretliyoruz
    robot_description = ParameterValue(
        # tek string: "xacro <dosya_yolu>"
        Command(['xacro ' + urdfModelPath]),
        value_type=str
    )
    params = {'robot_description': robot_description}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[params],
        # artık argüman yok, işlenmiş URDF parametresini kullanacak
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_gui'))
    )

    # Publish a static transform to lift the robot in RViz
    static_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_rise',
        output='screen',
        arguments=['0', '0', '0.2', '0', '0', '0', 'world', 'base_link']
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[params],
        arguments=['-d', rvizConfigPath]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Whether to start the joint state publisher GUI'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        static_tf_node,
        rviz_node
    ])

if __name__ == '__main__':
    generate_launch_description()