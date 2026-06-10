import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("pairs_softuav_simulation")
    model = os.path.join(share, "holoarm", "models", "holoarm_narrow_passage.xml")

    return LaunchDescription([
        Node(
            package="pairs_softuav_simulation",
            executable="drone_sim_node",
            name="drone_sim",
            output="screen",
            parameters=[{
                "model_path": model,
                "hover_thrust": 2.3789,
                "init_x": 0.0,
                "init_y": 0.0,
                "init_z": 1.0,
            }],
        ),
    ])
