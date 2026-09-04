from glob import glob

from setuptools import setup


package_name = "dog_patrol_perception_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HowAreAllWell",
    maintainer_email="77225398+HowAreAllWell@users.noreply.github.com",
    description="Standalone launch composition for the dog patrol perception stack.",
    license="BSD-3-Clause",
)
