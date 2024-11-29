from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pangolin_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruce',
    maintainer_email='BruceLin90620',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pangolin_control = pangolin_control.pangolin_control:main',
            # 'pangolin_action = pangolin_control.pangolin_control_action:main',
            'pangolin_state = pangolin_control.pangolin_state:main'
        ],
    },
)