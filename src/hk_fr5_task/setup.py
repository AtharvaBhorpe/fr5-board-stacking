from glob import glob

from setuptools import setup

package_name = "hk_fr5_task"

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
    description="hk_fr5 main control loop: world model, fake world, skills, task_main_controller.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "world_model = hk_fr5_task.world_model:main",
        "fake_world = hk_fr5_task.fake_world:main",
        "skill_server = hk_fr5_task.skill_server:main",
    ]},
)
