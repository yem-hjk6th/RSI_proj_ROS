from glob import glob

from setuptools import setup

package_name = "krc_eki_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/krc_templates", glob("krc_templates/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="EthernetKRL bridge for a KR C4 without RSI (override only, no motion targets).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "eki_bridge = krc_eki_bridge.node:main",
            "fake_eki_server = krc_eki_bridge.fake_eki:main",
        ],
    },
)
