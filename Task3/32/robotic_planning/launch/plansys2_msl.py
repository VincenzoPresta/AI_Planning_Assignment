import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('robotic_planning')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    fill_box_from_location_cmd = Node(
        package='robotic_planning',
        executable='fill_box_from_location_action_node',
        name='fill_box_from_location_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    fill_box_from_workstation_cmd = Node(
        package='robotic_planning',
        executable='fill_box_from_workstation_action_node',
        name='fill_box_from_workstation_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    empty_box_location_cmd = Node(
        package='robotic_planning',
        executable='empty_box_location_action_node',
        name='empty_box_from_location_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    empty_box_workstation_cmd = Node(
        package='robotic_planning',
        executable='empty_box_workstation_action_node',
        name='empty_box_workstation_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    pick_up_from_location_cmd = Node(
        package='robotic_planning',
        executable='pick_up_from_location_action_node',
        name='pick_up_from_location_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])    

    pick_up_from_workstation_cmd = Node(
        package='robotic_planning',
        executable='pick_up_from_workstation_action_node',
        name='pick_up_from_workstation_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])    

        
    move_cmd = Node(
        package='robotic_planning',
        executable='move_action_node',
        name='move_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    deliver_to_location_cmd = Node(
        package='robotic_planning',
        executable='deliver_to_location_action_node',
        name='deliver_to_location_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])    


    deliver_to_workstation_cmd = Node(
        package='robotic_planning',
        executable='deliver_to_workstation_action_node',
        name='deliver_to_workstation_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])    

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(fill_box_from_location_cmd)
    ld.add_action(fill_box_from_workstation_cmd)
    ld.add_action(empty_box_location_cmd)
    ld.add_action(empty_box_workstation_cmd)
    ld.add_action(pick_up_from_location_cmd)
    ld.add_action(pick_up_from_workstation_cmd)
    ld.add_action(move_cmd)
    ld.add_action(deliver_to_location_cmd)
    ld.add_action(deliver_to_workstation_cmd)


    
    return ld








