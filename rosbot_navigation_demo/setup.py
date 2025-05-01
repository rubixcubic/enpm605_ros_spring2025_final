from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rosbot_navigation_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        # Include launch files
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zeid",
    maintainer_email="adon.sf4@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cube_publisher_demo = rosbot_navigation_demo.cube_publisher_demo:main",
            "pid_controller_demo = rosbot_navigation_demo.pid_controller_demo:main",
            "color_beacon_demo = rosbot_navigation_demo.color_beacon_demo:main",
            "get_goal_service_demo = rosbot_navigation_demo.get_goal_service_demo:main",
            "move_to_goal_action_demo = rosbot_navigation_demo.move_to_goal_action_demo:main",
            "navigation_manager_demo = rosbot_navigation_demo.navigation_manager_demo:main",
        ],
    },
)
