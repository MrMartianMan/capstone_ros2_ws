from setuptools import setup
import os
from glob import glob

package_name = 'sprayer_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gage',
    maintainer_email='gmart64@lsu.edu',
    description='YOLO detection to sprayer trigger',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = sprayer_vision.yolo_detector_node:main',
            'sprayer_gpio_node = sprayer_vision.sprayer_gpio_node:main',
        ],
    },
)
