from setuptools import find_packages, setup

package_name = "fleet_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ziling",
    maintainer_email="ziling@zephastra.local",
    description="009 adapter contract and safety gate policy. No ROS dependency, by design.",
    license="Apache-2.0",
)
