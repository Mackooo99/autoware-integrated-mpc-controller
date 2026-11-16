from setuptools import setup
from glob import glob
import os

package_name = "control_benchmark_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nit-user",
    maintainer_email="nit-user@example.com",
    description="Monitoring and benchmark logger for Autoware control experiments.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "logger_node = control_benchmark_monitor.logger_node:main",
        ],
    },
)

