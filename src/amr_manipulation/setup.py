from setuptools import setup

package_name = 'amr_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trieu',
    maintainer_email='trieu@example.com',
    description='AMR manipulation and lift control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_splitter = amr_manipulation.cmd_vel_splitter:main',
            'lift_controller = amr_manipulation.lift_controller:main',
        ],
    },
)
