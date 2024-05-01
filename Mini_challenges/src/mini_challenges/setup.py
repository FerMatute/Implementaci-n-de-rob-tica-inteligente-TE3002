import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mini_challenges'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardon',
    maintainer_email='ricardonavarro2003@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_id = mini_challenges.color_id:main',
            'odometry_node = mini_challenges.odometry_node:main',
            'path_generator_c4 = mini_challenges.path_generator_c4:main',
        ],
    },
)
