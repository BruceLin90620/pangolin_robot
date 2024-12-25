from setuptools import setup
import os
from glob import glob

package_name = 'pangolin_visual_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Description of your package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pangolin_visual_nav = pangolin_visual_nav.pangolin_visual_nav:main',
            'goal_pose_visualization = pangolin_visual_nav.goal_pose_visualization:main',
        ],
    },
)
