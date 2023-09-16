import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get all the nodes from executable
    simple_publisher = Node(
        package     = 'adisha_test',
        executable  = 'simple_publisher'
    )
    simple_subscriber = Node(
        package     = 'adisha_test',
        executable  = 'simple_subscriber'
    )

    # Launch all
    return LaunchDescription([
        simple_publisher,
        simple_subscriber
    ])