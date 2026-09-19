from glob import glob

from setuptools import find_packages, setup

package_name = 'beetlebot_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BeetleBot project',
    maintainer_email='devapp489@gmail.com',
    description='Gazebo Harmonic simulation of the VEEROBOT BeetleBot.',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
