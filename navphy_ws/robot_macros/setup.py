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
    maintainer='adam',
    maintainer_email='sdpteam12.2023@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cambop = robot_macros:camera_bop:main',
            'rotatechassis = robot_macros:rotate_chassis:main'
        ],
    },
)
