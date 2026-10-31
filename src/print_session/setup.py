from glob import glob

from setuptools import find_packages, setup

package_name = "print_session"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="Print session lifecycle, rosbag2 recording and export to the legacy logs.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "session_node = print_session.session_node:main",
            "bag_to_legacy = print_session.bag_to_legacy:main",
        ],
    },
)
