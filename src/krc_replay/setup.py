from glob import glob

from setuptools import find_packages, setup

package_name = "krc_replay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="End-to-end replay of a logged session through the ROS graph.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "log_player = krc_replay.log_player:main",
        ],
    },
)
