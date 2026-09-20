from setuptools import find_packages, setup

package_name = "fleet_ros"

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
    description="ROS 2 nodes for the 009 fleet: safety gate and stop distance calibration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # The only publisher of <ns>/cmd_vel.
            "gate_node = fleet_ros.gate_node:main",
            # Measures the real stopping distance through the gate. Exit 3 means the
            # gate refused, which is a result, not a failure to retry.
            "stop_distance_calibrator = fleet_ros.stop_distance_calibrator:main",
            # Owns the single reservation book and grants passage permits.
            # Publishes no cmd_vel: it grants, it does not drive.
            "coordinator_node = fleet_ros.coordinator_node:main",
            # Drives the staged crossing: wait -> acquire -> enter -> exit ->
            # release -> confirm clear. Uses Nav2 actions; never cmd_vel.
            "staged_crossing = fleet_ros.staged_crossing:main",
        ],
    },
)
