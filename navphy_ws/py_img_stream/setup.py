from setuptools import find_packages, setup

package_name = 'py_img_stream'

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
            'pub = '
            'py_img_stream.image_pub:main',
            'sub = '
            'py_img_stream.image_test_sub:main',
            'target_pub = '
            'py_img_stream.target_pub:main',
            'img_pub_audio = '
            'py_img_stream.image_pub_audio:main'
        ],
    },
)
