from setuptools import setup

package_name = 'amr_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/slam.yaml',
        ]),
        ('share/' + package_name + '/maps', [
            'maps/amr_map.yaml',
            'maps/amr_map.pgm',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/slam.launch.py',
            'launch/slam_bringup.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/slam.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trieu',
    maintainer_email='trieu@example.com',
    description='AMR 2D SLAM using slam_toolbox',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
