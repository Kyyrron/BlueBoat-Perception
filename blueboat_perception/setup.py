from glob import glob

from setuptools import find_packages, setup

package_name = "blueboat_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    # the fixed detector-input mapping parameters (decision 6.4)
    package_data={"blueboat_perception.core": ["data/*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Killian BERTRAND",
    maintainer_email="killian.bertrand2002@gmail.com",
    description="Seabed-image perception for the BlueBoat side-scan survey",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = blueboat_perception.ros.perception_node:main",
            "blueboat-perception-offline = blueboat_perception.cli.offline:main",
        ],
    },
)
