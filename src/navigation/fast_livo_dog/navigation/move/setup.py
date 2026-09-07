import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz_cfg'),
            glob('rviz_cfg/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yang',
    maintainer_email='yang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = move.pure_pursuit:main',
            'priest_rl_publisher_nav_cmd = move.priest_rl_publisher_nav_cmd:main',
            'priest_rl_publisher_nav_cmd_fast = move.priest_rl_publisher_nav_cmd_fast:main',
            (
                'priest_mppi_adapter_nav_cmd_dwb_smooth_responsive = '
                'move.priest_mppi_adapter_nav_cmd_dwb_smooth_responsive:main'
            ),
            'global_path_publisher = move.global_path_publisher:main',
            'global_path_seq_publisher = move.global_path_seq_publisher:main',
            'navigation_path_mux = move.navigation_path_mux:main',
            'nav_cmd_domain_bridge = move.nav_cmd_domain_bridge:main',
        ],
    },
)
