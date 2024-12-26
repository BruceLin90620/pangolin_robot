from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apriltag_localize'

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
    extras_require={  
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='bruce',
    maintainer_email='bruce@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'apriltag_localization_node = apriltag_localize.apriltag_localize:main',
        ],
    },
)
