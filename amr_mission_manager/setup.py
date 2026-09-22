from setuptools import setup
import os
from glob import glob

package_name = 'amr_mission_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Trieu',
    maintainer_email='trieu@example.com',
    description='AMR Mission Manager - Route navigation mission management',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager = amr_mission_manager.mission_manager_node:main',
            'mission_gui = amr_mission_manager.mission_gui:main',
        ],
    },
)
