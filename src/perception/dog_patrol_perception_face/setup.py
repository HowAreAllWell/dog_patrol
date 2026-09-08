from glob import glob

from setuptools import find_packages, setup


package_name = "dog_patrol_perception_face"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml", "README.md", "COLLABORATION.md", "LICENSE"],
        ),
        (f"share/{package_name}/config", ["config/face.yaml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy>=1.23,<3",
        "PyYAML>=6.0,<7",
        "opencv-python>=4.5",
    ],
    zip_safe=True,
    maintainer="HowAreAllWell",
    maintainer_email="77225398+HowAreAllWell@users.noreply.github.com",
    description="Face verification integration boundary for dog_patrol.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_face_provider = dog_patrol_perception_face.provider:main",
            "perception_face_readiness = dog_patrol_perception_face.readiness_node:main",
            "perception_face_enroll = dog_patrol_perception_face.enrollment:main",
        ],
    },
)
