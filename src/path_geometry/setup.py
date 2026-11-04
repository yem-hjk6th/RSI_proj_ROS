from glob import glob

from setuptools import find_packages, setup

package_name = "path_geometry"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/patterns", glob("patterns/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="Fan and semicircle pattern geometry, and the angle failure map tool (no ROS imports).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "angle_failure_map = path_geometry.cli:main",
        ],
    },
)
