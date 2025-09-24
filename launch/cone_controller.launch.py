from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_control',
            executable='cone_controller',
            name='cone_controller',
            output='screen',
            parameters=[{
                # controller gains/limits
                'ctrl_freq': 50.0,
                'kv': 0.5,
                'kw': 1.5,
                'v_min': -0.5,
                'v_max': 2.0,
                'w_min': -1.0,
                'w_max': 1.0,
                # topics/types (poseS = PoseStamped, pose2d = Pose2D)
                'goal_topic': '/simple_goal',
                'goal_type': 'poseS',
                'odom_topic': '/odom',
                'odom_type': 'poseS',
            }],
        ),
    ])
