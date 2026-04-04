import os
from glob import glob

from setuptools import setup

package_name = 'robot_sync_sim'

setup(
    name=package_name,
    version='1.0.0',
    py_modules=['leader_node', 'follower_node'],
    package_dir={'': 'scripts'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models', 'trash'), glob('models/trash/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='maintainer@example.com',
    description='Gazebo leader-follower trash pickup simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader_node = leader_node:main',
            'follower_node = follower_node:main',
        ],
    },
)
