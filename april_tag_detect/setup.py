from setuptools import setup
import os
from glob import glob

package_name = 'april_tag_detect'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'flask',
        'apriltag'
    ],
    zip_safe=True,
    maintainer='bjh',
    maintainer_email='bjhee0430@gmail.com',
    description='AprilTag detection node with Flask streaming',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'april_tag_node = april_tag_node:main',
        ],
    },
)
