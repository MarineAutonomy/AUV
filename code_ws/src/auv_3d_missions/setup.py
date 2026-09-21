import os
from glob import glob

from setuptools import find_packages, setup

package_name = "auv_3d_missions"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        *[
            (os.path.join("share", package_name, os.path.dirname(p)), [p])
            for p in glob("config/**/*.yaml", recursive=True)
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aatmaj",
    maintainer_email="na22b018@smail.iitm.ac.in",
    description="Mission-level guidance nodes for 3D AUV autonomy.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "point_tracking_mission_3d_ilos = auv_3d_missions.point_tracking.ilos:main",
            "point_tracking_mission_3d_los = auv_3d_missions.point_tracking.los:main",
            "depth_control_mission_3d = auv_3d_missions.depth_control.hover:main",
            "station_keeping_mission_3d = auv_3d_missions.station_keeping.hold:main",
            "dwell_mission_3d = auv_3d_missions.dwell.roll_dwell:main",
        ],
    },
)

