from setuptools import setup


package_name = "tank_base"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/base_driver.launch.py"]),
        ("share/" + package_name + "/config", ["config/serial.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sgj",
    maintainer_email="sgj@example.com",
    description="Serial base driver for chassis control and feedback.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "chassis_driver_node = tank_base.chassis_driver_node:main",
        ],
    },
)
