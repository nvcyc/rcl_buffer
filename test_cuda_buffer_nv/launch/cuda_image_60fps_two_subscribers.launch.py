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
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topic_name = LaunchConfiguration('topic_name')
    publish_rate_ms = LaunchConfiguration('publish_rate_ms')
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    log_every_n = LaunchConfiguration('log_every_n')

    publisher = Node(
        package='test_cuda_buffer_nv',
        executable='cuda_image_60fps_publisher_node',
        name='cuda_image_60fps_publisher',
        output='screen',
        parameters=[{
            'topic_name': topic_name,
            'publish_rate_ms': ParameterValue(publish_rate_ms, value_type=int),
            'image_width': ParameterValue(image_width, value_type=int),
            'image_height': ParameterValue(image_height, value_type=int),
            'max_publish_count': 0,
            'expected_subscription_count': 2,
            'log_every_n': ParameterValue(log_every_n, value_type=int),
        }],
    )

    any_backend_subscriber = Node(
        package='test_cuda_buffer_nv',
        executable='cuda_image_60fps_subscriber_node',
        name='cuda_image_60fps_any_backend_subscriber',
        output='screen',
        parameters=[{
            'topic_name': topic_name,
            'expected_backend': 'cuda',
            'acceptable_buffer_backends': 'any',
            'log_every_n': ParameterValue(log_every_n, value_type=int),
        }],
    )

    cuda_only_subscriber = Node(
        package='test_cuda_buffer_nv',
        executable='cuda_image_60fps_subscriber_node',
        name='cuda_image_60fps_cuda_only_subscriber',
        output='screen',
        parameters=[{
            'topic_name': topic_name,
            'expected_backend': 'cuda',
            'acceptable_buffer_backends': 'cuda',
            'log_every_n': ParameterValue(log_every_n, value_type=int),
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        DeclareLaunchArgument(
            'topic_name',
            default_value='cuda_image_60fps',
            description='Image topic used by the CUDA publisher and subscribers.',
        ),
        DeclareLaunchArgument(
            'publish_rate_ms',
            default_value='17',
            description='Publisher timer period in milliseconds. 17 ms is about 60 fps.',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Published image width in pixels.',
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Published image height in pixels.',
        ),
        DeclareLaunchArgument(
            'log_every_n',
            default_value='60',
            description='Log every N published/received frames. Set to 0 to disable frame logs.',
        ),
        publisher,
        any_backend_subscriber,
        cuda_only_subscriber,
    ])
