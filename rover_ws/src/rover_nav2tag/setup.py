from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rover_nav2tag"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Space Bulls",
    maintainer_email="june.bang.h@gmail.com",
    description=(
        "DEIMOS Nav2 AprilTag navigation: filters TF tags by requested ID "
        "and commands Nav2 to navigate to a standoff pose in front of the tag."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nav2tag_node = rover_nav2tag.nav2tag_node:main",
            "fake_tag_publisher = rover_nav2tag.fake_tag_publisher:main",
        ],
    },
)
