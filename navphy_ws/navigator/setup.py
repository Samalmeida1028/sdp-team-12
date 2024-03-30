from setuptools import find_packages, setup

package_name = 'navigator'

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
    maintainer='Arjun Viswanathan',
    maintainer_email='arjun.viswanathan2612@gmail.com',
    description='Testing out navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2pose = navigator.nav2pose:main',
            'sens2odom = navigator.sens2odom:main',
            'searchtargets = navigator.search_targets:main',
            'filterlidar = navigator.filter_lidar:main'
        ],
    },
)