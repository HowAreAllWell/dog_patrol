from glob import glob

from setuptools import find_packages, setup


package_name = "dog_patrol_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/docs", glob("docs/*.md")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zby",
    maintainer_email="1193040110@qq.com",
    description="Python navigation integration for the dog patrol mission contract.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigation_mission_coordinator = "
            "dog_patrol_navigation.navigation_mission_coordinator:main",
        ],
    },
)
