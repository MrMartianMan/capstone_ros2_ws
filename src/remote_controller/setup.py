import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'remote_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ee470212',
    maintainer_email='ee470212@todo.todo',
    description='Remote controller package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = remote_controller.teleop_node:main',
            'mode_manager_node = remote_controller.mode_manager_node:main',
        ],
    },
)
