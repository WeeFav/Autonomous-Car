from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo

def generate_launch_description():
    ld = LaunchDescription()

    # find path to package
    orbslam_nav_sim_pkg_path = FindPackageShare('orbslam_nav_sim')
    
    # join paths together
    urdf_path = PathJoinSubstitution([orbslam_nav_sim_pkg_path, 'urdf', 'my_robot.urdf.xacro'])

    # publish tf
    robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command(['xacro',' ', urdf_path])
            }]
        )
    ld.add_action(robot_state_publisher_node)
    
    # launch gazebo using gazebo launch file
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            "world": PathJoinSubstitution([orbslam_nav_sim_pkg_path, 'worlds', 'small_house.world']),
            "use_sim_time": "true"
        }.items()
    ))
    
    # spawn the robot in gazebo
    spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', 'my_robot',
                '-x', '4.0',
                '-y', '0.0',
                '-z', '0.0',
            ]
        )
    ld.add_action(spawn_entity_node)
    
    # # launch rviz
    # rviz_config_path = PathJoinSubstitution([orbslam_nav_sim_pkg_path, 'rviz', 'urdf.rviz'])
    # rviz2_node = Node(
    #         package="rviz2", 
    #         executable="rviz2",
    #         output="screen",
    #         arguments=["-d", rviz_config_path]
    #     )
    # ld.add_action(rviz2_node)
    
    return ld
