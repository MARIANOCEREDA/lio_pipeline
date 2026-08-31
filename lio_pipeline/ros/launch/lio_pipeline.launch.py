from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/data",
        description="Input IMU topic.",
    )

    pcl_topic_arg = DeclareLaunchArgument(
        "pcl_topic",
        default_value="/points",
        description="Input PointCloud2 topic.",
    )

    lio_pipeline_node = LifecycleNode(
        package="lio_pipeline",
        executable="lio_pipeline_node",
        name="lio_pipeline_node",
        namespace="",
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'pcl_topic': LaunchConfiguration('pcl_topic'),
            }
        ],
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_lio_pipeline",
        output="screen",
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['lio_pipeline_node'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            imu_topic_arg,
            pcl_topic_arg,
            lio_pipeline_node,
            lifecycle_manager,
        ]
    )