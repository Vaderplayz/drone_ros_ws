from glob import glob
from setuptools import find_packages, setup


package_name = "mini_ground_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements.txt", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/icons", glob("icons/*.svg")),
        ("share/" + package_name + "/desktop", glob("desktop/*.desktop.in")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="le-hai-trung",
    maintainer_email="lehaitrung@todo.todo",
    description="Low-latency PySide6 ground-control display for ROS 2 and PX4.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mini_ground_control = mini_ground_control.main:main",
            "mini_ground_control_mock = mini_ground_control.mock_publisher:main",
            "pipeline_supervisor = mini_ground_control.pipeline_supervisor:main",
        ],
    },
)
