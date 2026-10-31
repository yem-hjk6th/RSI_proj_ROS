from glob import glob

from setuptools import find_packages, setup

package_name = "krc_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/patterns", glob("patterns/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="Fake KUKA KR C4 RSI client: 4 ms packets, motion along a pattern at programmed speed x override, late-reply counting with the RSI Timeout, replay of recorded RSI logs.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sim_node = krc_sim.sim_node:main",
        ],
    },
)
