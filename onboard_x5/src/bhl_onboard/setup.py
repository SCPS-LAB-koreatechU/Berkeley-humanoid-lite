import os
from glob import glob

from setuptools import setup

package_name = "bhl_onboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SCPS Lab",
    maintainer_email="hexk0131@gmail.com",
    description="Berkeley Humanoid Lite on-board low-level controller (RDK X5)",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bhl_lowlevel = bhl_onboard.lowlevel_node:main",
            "bhl_commander = bhl_onboard.commander_node:main",
            "bhl_lowcmd_example = bhl_onboard.lowcmd_example:main",
        ],
    },
)
