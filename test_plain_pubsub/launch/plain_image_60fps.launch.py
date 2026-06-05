# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topic_name = LaunchConfiguration('topic_name')
    publish_rate_ms = LaunchConfiguration('publish_rate_ms')
    log_every_n = LaunchConfiguration('log_every_n')

    publisher = Node(
        package='test_plain_pubsub',
        executable='plain_image_publisher_node',
        name='plain_image_60fps_publisher',
        output='screen',
        parameters=[{
            'topic_name': topic_name,
            'publish_rate_ms': ParameterValue(publish_rate_ms, value_type=int),
            'max_publish_count': 0,
            'log_every_n': ParameterValue(log_every_n, value_type=int),
        }],
    )

    subscriber = Node(
        package='test_plain_pubsub',
        executable='plain_image_subscriber_node',
        name='plain_image_60fps_subscriber',
        output='screen',
        parameters=[{
            'topic_name': topic_name,
            'log_every_n': ParameterValue(log_every_n, value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'topic_name',
            default_value='plain_image_60fps',
            description='Image topic used by the publisher and subscriber.',
        ),
        DeclareLaunchArgument(
            'publish_rate_ms',
            default_value='17',
            description='Publisher timer period in milliseconds. 17 ms is about 60 fps.',
        ),
        DeclareLaunchArgument(
            'log_every_n',
            default_value='60',
            description='Log every N published/received frames. Set to 0 to disable frame logs.',
        ),
        publisher,
        subscriber,
    ])
