from setuptools import setup
import os
from glob import glob

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Çağatay Altıntopaç',
    maintainer_email='cagatay@example.com',
    description='My Bot package for autonomous navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go_to_point = my_bot.go_to_point:main',
            'astar_planner = my_bot.astar_planner:main',
            'path_follower = my_bot.path_follower:main',
        ],
    },
)
