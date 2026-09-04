from setuptools import setup

package_name = 'robot_console'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zby',
    maintainer_email='1193040110@qq.com',
    description='System-level Qt control console for dog patrol orchestration.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_console = robot_console.app:main',
        ],
    },
)
