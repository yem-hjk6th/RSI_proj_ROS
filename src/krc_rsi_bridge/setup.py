from glob import glob

from setuptools import find_packages, setup

package_name = "krc_rsi_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="KUKA RSI UDP server for the KR C4: 4 ms reply loop in its own process, override clamp, speed floor, heartbeat watchdog with fallback ramp; publishes KrcState, subscribes to OverrideCmd.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = krc_rsi_bridge.bridge_node:main",
        ],
    },
)
