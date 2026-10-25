from glob import glob

from setuptools import setup

package_name = "hk_fr5_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vortexcv",
    maintainer_email="69069296+vortexcv@users.noreply.github.com",
    description="hk_fr5 motion nodes: motion_controller (move_to, move_delta, get_state), obstacle_loader.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "obstacle_loader = hk_fr5_motion.obstacle_loader:main",
            "motion_controller = hk_fr5_motion.motion_controller:main",
        ],
    },
)
