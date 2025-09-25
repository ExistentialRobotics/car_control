# ROS2 Car Control

This package contains a single node that perform local control driving a differential drive robot to a desired state.

The power diagram controller has been removed since it uses the same control law as the cone controller.

The DD controller shares the same finite-state-machine as the cone controller. 

**Using the cone controller is highly recommended.**

Example ROS2 launch file:

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_control',
            executable='cone_controller',
            name='cone_controller',
            output='screen',
            parameters=[{
                # controller general setup, available modes: 'cone', 'dd'
                'mode': 'cone',
                'ctrl_freq': 50.0,
                # controller gains
                'kv': 0.5,
                'kw': 1.5,
                'dd_ka': 8.0,
                'dd_kb': -1.5,
                # controller bounds
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
            remappings=[
                ('/cmd_vel', '/jackal1/cmd_vel'),
            ]
        ),
    ])
```
The parameter `kv` controls the linear gain in both `cone` and `dd` modes. 
The parameter `kw` controls the angular gain in `cone` mode.

The parameters `dd_ka` and `dd_kb` are only valid when operating in `dd` mode. 
Those two parameters jointly control the angular gain in `dd` mode.

Once the package has been built, modify the launch file located at: `/install/car_control/share/car_controll/launch` 
to have immediate effect.

