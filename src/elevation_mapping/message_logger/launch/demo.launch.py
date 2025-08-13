from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "automatic_error_reporting_disable",
                default_value="false",
                description="Disable automatic error reporting",
            ),
            DeclareLaunchArgument(
                "automatic_error_reporting_min_breadcrumb_level",
                default_value="info",
                description="Minimum breadcrumb level for automatic error reporting",
            ),
            # Set configuration parameters directly
            SetLaunchConfiguration(name="error_reporting", value="automatic_full"),
            # Define the node with parameters
            Node(
                package="message_logger",
                executable="demo",
                name="demo",
                output="screen",
                parameters=[
                    {
                        "automatic_error_reporting/disable": LaunchConfiguration(
                            "automatic_error_reporting_disable"
                        ),
                        "automatic_error_reporting/min_breadcrumb_level": LaunchConfiguration(
                            "automatic_error_reporting_min_breadcrumb_level"
                        ),
                        "/config/feature_toggles/error_reporting": "automatic_full",
                    }
                ],
            ),
        ]
    )
