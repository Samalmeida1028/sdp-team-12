from setuptools import find_packages, setup

package_name = 'marker_detection'

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
    maintainer='arjun',
    maintainer_email='arjun.viswanathan2612@gmail.com',
    description='Marker detection package to detect ArUco markers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = marker_detection.MarkerDetection:main',
            'tester = marker_detection.TargetPublisher:main',
        ],
    },
)
