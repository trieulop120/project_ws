from setuptools import setup

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/nav2.launch.py',
            'launch/nav2_bringup.launch.py',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/navigation.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trieu',
    maintainer_email='trieu@example.com',
    description='AMR navigation using Nav2 with EKF localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)
