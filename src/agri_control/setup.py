from setuptools import find_packages, setup

package_name = 'agri_control'

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
    maintainer='ee470212',
    maintainer_email='ee470212@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
     'joy_simulator = agri_control.joy_simulator_node:main',
     'joy_evdev_node = agri_control.joy_evdev_node:main',
 ],
    },
)
