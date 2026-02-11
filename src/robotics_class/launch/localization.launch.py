import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Caminhos
    pkg_share = get_package_share_directory('robotics_class')
    
    # Argumentos de Launch
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'class_map.yaml'),
        description='Caminho completo para o arquivo yaml do mapa')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'params', 'default2.yaml'),
        description='Caminho completo para o arquivo de parametros')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    autostart_arg = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    # Variáveis
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # 1. Nó do Map Server (Carrega o mapa estático)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, 
                    {'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time}]
    )

    # 2. Nó do AMCL (Localização Probabilística)
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time}]
        # O remapping do scan já está sendo feito via default.yaml na linha scan_topic
    )

    # 3. Lifecycle Manager (Gerente que ativa os nós na ordem certa)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        map_file_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])