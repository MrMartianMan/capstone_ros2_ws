from setuptools import setup

package_name = 'crop_row_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Crop row centerline extraction',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'centerline_node = crop_row_perception.centerline_node:main',
        ],
    },
)
