from glob import glob

from setuptools import find_packages, setup


package_name = "dog_patrol_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zby",
    maintainer_email="1193040110@qq.com",
    description="Global mission supervisor for the M20 patrol workflow.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_supervisor = dog_patrol_manager.mission_supervisor:main",
        ],
    },
)
