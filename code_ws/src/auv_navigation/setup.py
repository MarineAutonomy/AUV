import os
from glob import glob

from setuptools import find_packages, setup

package_name = "auv_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aatmaj",
    maintainer_email="na22b018@smail.iitm.ac.in",
    description="AUV navigation EKF (stable numerics, same topics as auv_navigation)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigation_node = navigation.navigation_node:main",
            "imu_enu_to_ned = navigation.imu_enu_to_ned:main",
        ],
    },
)
