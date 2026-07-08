from setuptools import find_packages, setup

package_name = 'rover_core'

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
    maintainer='rptech',
    maintainer_email='rptech@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'rover_teleop = rover_core.rover_teleop:main',
            'rover_teleop_v2 = rover_core.rover_teleop_v2:main',
            'rover_odometry = rover_core.rover_odometry:main',
        ],
    },
)
