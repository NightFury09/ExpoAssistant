import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rptech',
    maintainer_email='rptech@todo.todo',
    description='Robot URDF description package for Rover',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
