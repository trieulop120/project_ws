from setuptools import setup

package_name = 'amr_mission_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trieu',
    maintainer_email='trieu@example.com',
    description='AMR Mission Manager - Mission execution and route planning',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_manager = amr_mission_manager.mission_manager_node:main',
        ],
    },
)
