import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thienvu',
    maintainer_email='thienvu@todo.todo',
    description='AMR Hardware Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster = amr_hardware.static_tf_broadcaster:main',
            'esp32_bridge = amr_hardware.esp32_bridge:main', # [GIỮ NGUYÊN]: Đã dùng node esp32_bridge duy nhất
            'teleop_amr = amr_hardware.teleop_amr:main',
            'cli_control = amr_hardware.cli_control:main',
            'cmd_vel_splitter = amr_hardware.cmd_vel_splitter:main',
        ],
    },
)