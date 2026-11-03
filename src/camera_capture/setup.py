from glob import glob

from setuptools import find_packages, setup

package_name = "camera_capture"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="Camera capture and recording (V4L2, ffmpeg HEVC / MJPG AVI / mp4v), image and preview topics.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node = camera_capture.node:main",
            "preview_node = camera_capture.preview_node:main",
            "camera_capture_verify = camera_capture.verify:main",
            "camera_capture_bench = camera_capture.bench:main",
        ],
    },
)
