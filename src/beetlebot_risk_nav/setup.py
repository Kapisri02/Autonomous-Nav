from glob import glob

from setuptools import find_packages, setup

package_name = 'beetlebot_risk_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/beetlebot_nav.yaml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BeetleBot project',
    maintainer_email='devapp489@gmail.com',
    description='Autonomous start-to-goal navigation for the VEEROBOT BeetleBot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beetlebot_nav = beetlebot_risk_nav.nodes.beetlebot_nav_node:main',
        ],
    },
)
