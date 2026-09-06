from setuptools import setup

setup(
    name='amr_localization',
    version='0.1.0',
    packages=['amr_localization'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/amr_localization']),
        ('share/amr_localization', ['package.xml']),
        ('share/amr_localization/config', [
            'config/ekf.yaml',
        ]),
        ('share/amr_localization/launch', [
            'launch/ekf.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Trieu',
    maintainer_email='trieu@example.com',
    description='AMR localization: EKF odometry, SLAM, AMCL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
