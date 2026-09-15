from glob import glob

from setuptools import find_packages, setup

package_name = 'map_selection_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        # The map is installed into the package share directory so that it can
        # be located with get_package_share_directory() instead of an absolute
        # path into somebody's home directory. A glob is used deliberately: the
        # .pgm is a binary asset, and the build must not fail outright if it has
        # not been copied in yet - the node reports the missing image instead.
        (
            'share/' + package_name + '/maps',
            glob('maps/*.yaml') + glob('maps/*.pgm')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kapisri02',
    maintainer_email='kapisri02@todo.todo',
    description='Map display and endpoint (goal) selection for the BeetleBot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_selection_node = '
            'map_selection_test.map_selection_node:main',
        ],
    },
)
