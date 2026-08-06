from setuptools import find_packages, setup


package_name = "dog_patrol_perception_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md", "LICENSE", "requirements.txt"]),
        (f"share/{package_name}/config", ["config/voice.yaml"]),
        (f"share/{package_name}/tools", ["tools/r818_pcm_base64.c"]),
    ],
    package_data={package_name: ["assets/r818_pcm_base64_aarch64"]},
    install_requires=[
        "setuptools",
        "numpy>=1.23,<3",
        "PyYAML>=6.0,<7",
        "vosk==0.3.45",
    ],
    zip_safe=False,
    maintainer="HowAreAllWell",
    maintainer_email="77225398+HowAreAllWell@users.noreply.github.com",
    description="Portable task-scoped R818/Vosk voice verification core for dog_patrol.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
)
