from glob import glob

from setuptools import find_packages, setup

package_name = "width_readout_bore1"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.*")),
        *[("share/" + package_name + "/" + d, glob(d + "/*.*"))
          for d in ("config/bore_weights/deploy_bore1_cage2ft_261103",
                    "config/bore_weights/deploy_bore1_cage2ft_261103/weights",
                    "config/bore_weights/deploy_bore2_cage2ft_261103",
                    "config/bore_weights/deploy_bore2_cage2ft_261103/weights")],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yem-hjk6th",
    maintainer_email="183082151+yem-hjk6th@users.noreply.github.com",
    description="bore1 bead-width readout: image + KrcState -> Width (torch or replay head).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "width_readout_bore1_node = width_readout_bore1.node:main",
            "width_readout_bore1_offline = width_readout_bore1.cli:main",
        ],
    },
)
