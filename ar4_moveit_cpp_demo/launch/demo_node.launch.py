from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    demo_node = Node(
        package='ar4_moveit_cpp_demo',
        executable='simple_pick_place_mgi',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
            }
        ],
        additional_env={'RCUTILS_LOGGING_MIN_SEVERITY': 'INFO'},
    )

    return LaunchDescription([demo_node])
