from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    race_track_maker_pkg = get_package_share_directory("race_track_maker")
    race_track_name_arg = DeclareLaunchArgument(name="race_track", default_value="race_track")

    race_track_name_path = PathJoinSubstitution([
            race_track_maker_pkg,
            "tracks",
            PythonExpression(expression=["'", LaunchConfiguration("race_track"), "'", " + '.csv'"])
        ]
    )

    print(race_track_name_path)
    transform_frame = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'base_link'],
            output='screen',
        )
    
    path_publisher_node = Node(
            package='race_track_maker',
            executable='path_publisher.py',
            name='path_publisher',
            output='screen',
            parameters=[
                {'csv_file': race_track_name_path}  # You can specify the CSV file path here
            ],
        )
    return LaunchDescription([
        race_track_name_arg,

        transform_frame,
        path_publisher_node,
    ])