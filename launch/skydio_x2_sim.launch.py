import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # NOTE: indoor_env.xml includes the Skydio X2 model from mujoco_menagerie,
    # which is not installed by colcon. Run this target from the source tree
    # (with the menagerie downloaded under skydio_x2/models/), or copy the
    # menagerie into the installed share/ directory.
    share = get_package_share_directory("pairs_softuav_simulation")
    model = os.path.join(share, "skydio_x2", "models", "indoor_env.xml")

    return LaunchDescription([
        Node(
            package="pairs_softuav_simulation",
            executable="drone_sim_node",
            name="drone_sim",
            output="screen",
            parameters=[{
                "model_path": model,
                "hover_thrust": 3.2495,
                "init_x": 3.0,
                "init_y": 5.0,
                "init_z": 1.0,
            }],
        ),
    ])
