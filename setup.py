import os
from glob import glob
from setuptools import setup

package_name = "pairs_softuav_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # Install the MJCF models alongside the package so launch files can find them.
        (os.path.join("share", package_name, "holoarm", "models"),
            glob("holoarm/models/*.xml")),
        (os.path.join("share", package_name, "skydio_x2", "models"),
            glob("skydio_x2/models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="PAIRS Lab",
    maintainer_email="canhthanhlt@gmail.com",
    description="MuJoCo soft-UAV simulation with a ROS 2 wrapper node.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drone_sim_node = pairs_softuav_simulation.drone_sim_node:main",
        ],
    },
)
