#Antr - Autonomous Navigation and Tracking Robot 
#The Simulation bringup file for the Antr robot

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    antr_description = os.path.join(get_package_share_path('antr_description'))
    gazebo_description = os.path.join(get_package_share_path('ros_gz_sim'))
    # rviz_config_path = os.path.join(get_package_share_path('antr_description'),
    #                                 'rviz', 'urdf_config.rviz')
    
    mob_brignup = os.path.join(get_package_share_path('antr_bringup'))
    mob_launch = os.path.join(antr_description, 'launch', 'mob.launch.py')
    gz_launch = os.path.join(gazebo_description, 'launch', 'gz_sim.launch.py')
    world = os.path.join(mob_brignup, 'worlds', 'world1.sdf')
    rviz2_config_path = os.path.join(mob_brignup,"config","mob_rviz.rviz")

    robot_controllers = os.path.join(get_package_share_path('antr_description'),'config','controller_config.yaml',)
    antr_description_launch = IncludeLaunchDescription(launch_description_source = mob_launch)

    start_gazebo = IncludeLaunchDescription(launch_description_source = gz_launch,launch_arguments= {'gz_args': f"{world} -r",}.items())

    parameter_bridge_config = os.path.join(mob_brignup, 'config', 'gz_ros_topic_bridge.yaml')
    map_config = os.path.join(mob_brignup, 'config', 'map_config.yaml')

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
            arguments=[
                "-topic", "/robot_description",
        ],
        parameters=[{"/use_sim_time":True}]
    )
    
    map_tf =         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', '1','map', 'odom']
        )


    ros_gz_bridge = Node(
        package=  'ros_gz_bridge',
        executable="parameter_bridge",
        parameters=[
            {"config_file":parameter_bridge_config}]
    )
   
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz2_config_path]
    )

    joint_pub = Node(
        package="antr_controller",
        executable="control_robot",
    )

    localize_robot = Node(
        package="antr_controller",
        executable="robot_localize",
    )

    launch_map = Node(
        package="nav2_map_server",
        executable="map_server",
            parameters=[
            {"yaml_filename":map_config}]
    )

    nav2 = IncludeLaunchDescription(

        launch_description_source = os.path.join(get_package_share_path('nav2_bringup'),"bringup_launch.py"),

    )

    return LaunchDescription([
        antr_description_launch,
        start_gazebo,
        spawn_robot,
        ros_gz_bridge,
        rviz2_node,
        map_tf,
        # localize_robot,
        joint_pub,
        # launch_map
        # nav2
    ])