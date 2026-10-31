from glob import glob

from setuptools import find_packages, setup

package_name = "width_readout_elp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/rig", glob("config/rig/*")),
        ("share/" + package_name + "/config/pattern", glob("config/pattern/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="ELP chroma-edge bead-width readout: image + KrcState -> Width.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "width_readout_elp_node = width_readout_elp.node:main",
            "width_readout_elp_offline = width_readout_elp.cli:main",
        ],
    },
)
