import os
import glob
from setuptools import setup

package_name = "phantom_touch_description"
setup(
    name=package_name,
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share/",package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "meshes"), glob.glob("meshes/*.stl")),
        (os.path.join("share", package_name, "urdf"), glob.glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "state_publisher = phantom_touch_description.state_publisher:main"
        ],
    },
)
