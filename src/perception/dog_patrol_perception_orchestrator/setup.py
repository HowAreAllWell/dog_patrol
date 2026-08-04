from setuptools import find_packages, setup


package_name = "dog_patrol_perception_orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HowAreAllWell",
    maintainer_email="77225398+HowAreAllWell@users.noreply.github.com",
    description=(
        "ROS-independent perception business orchestration for dog_patrol."
    ),
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_readiness = dog_patrol_perception_orchestrator.readiness_node:main",
        ],
    },
)
