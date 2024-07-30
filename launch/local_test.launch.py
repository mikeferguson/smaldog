#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def generate_launch_description():
    # Load the URDF
    smaldog_dir = get_package_share_directory('smaldog')
    urdf_path = os.path.join(smaldog_dir, 'urdf', 'smaldog.urdf')
    urdf = open(urdf_path).read()

    return LaunchDescription([
        Node(
            name='smaldog',
            package='smaldog',
            executable='driver',
            parameters=[{'ip_address': '127.0.0.1'}],
            output='screen',
        ),
        Node(
            name='smaldog_loopback',
            package='smaldog',
            executable='loopback',
        ),
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf,
                         'publish_frequency': 100.0}],
        ),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
