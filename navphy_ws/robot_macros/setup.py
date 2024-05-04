from setuptools import find_packages, setup

package_name = 'robot_macros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdpteam12',
    maintainer_email='sdpteam12.2023@yahoo.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_bop = robot_macros.camera_bop:main',
            'rot_dance = robot_macros.rotate_dance:main'
        ],
    },
)
