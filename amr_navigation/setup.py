from setuptools import setup
import os
from glob import glob

package_name = 'amr_navigation'

# Find all config files in subdirectories
config_files = glob(os.path.join('config', '**', '*.yaml'), recursive=True)
config_files += glob(os.path.join('config', '**', '*.geojson'), recursive=True)
config_files = [f for f in config_files if os.path.isfile(f)]

# Find launch files
launch_files = glob(os.path.join('launch', '*.launch.py'))

# Find rviz files
rviz_files = glob(os.path.join('rviz', '*.rviz'))

setup(
    name=package_name,
    version='0.1.0',
    # Python modules in amr_navigation/ subdirectory
    py_modules=[
        'amr_navigation.interactive_node_creator',
        'amr_navigation.yaml_to_geojson',
        'amr_navigation.route_graph_publisher',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/config/graphs', config_files),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/rviz', rviz_files),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Trieu',
    maintainer_email='trieu@example.com',
    description='AMR navigation using Nav2 with EKF localization and Route Graph',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yaml_to_geojson = amr_navigation.yaml_to_geojson:main',
            'interactive_node_creator = amr_navigation.interactive_node_creator:main',
            'route_graph_publisher = amr_navigation.route_graph_publisher:main',
        ],
    },
)
