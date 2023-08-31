from setuptools import setup

package_name = "rclpy_osm_cartographer"
cartographer_package_name = package_name + ".cartographer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, cartographer_package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    package_dir={"": "src"},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="reidlo",
    maintainer_email="naru5135@wavem.net",
    description="ROS2(foxy) rclpy osm cartographer",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cartographer = rclpy_osm_cartographer.main:main"
        ],
    },
)
