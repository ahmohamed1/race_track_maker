from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    race_track_maker_pkg = get_package_share_directory("race_track_maker")
    
    # Declare the launch argument for the race track name without extension
    race_track_name_arg = DeclareLaunchArgument(name="race_track", default_value="v1")

    # Construct the full path with the .csv extension
    race_track_name_path = PathJoinSubstitution([
        race_track_maker_pkg,
        "tracks",
        PythonExpression(["'", LaunchConfiguration("race_track"), "'", " + '.csv'"])
    ])

    # Print the resolved path (for debugging purposes)
    print("Resolved CSV Path:", race_track_name_path)

    # Node to publish the static transform
    transform_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        output='screen',
    )
    
    # Node to publish the path using the resolved CSV file path
    path_publisher_node = Node(
        package='race_track_maker',
        executable='path_publisher.py',
        name='path_publisher',
        output='screen',
        parameters=[
            {'csv_file': race_track_name_path}  # Pass the constructed CSV file path
        ],
    )

    return LaunchDescription([
        race_track_name_arg,  # Include the race track name argument
        transform_frame,      # Include the transform frame node
        path_publisher_node,  # Include the path publisher node
    ])
