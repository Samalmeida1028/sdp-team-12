from setuptools import find_packages, setup

package_name = 'image_streaming'

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
    maintainer='sdp12',
    maintainer_email='sdp12@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
      'img_publisher = image_streaming.image_pub:main',
      'img_test_sub = image_streaming.image_test_sub:main',
    ],
    },
)
