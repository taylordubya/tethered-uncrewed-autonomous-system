import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'te_uas'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mdr',
    maintainer_email='jonathan.turnage@missiondrivenresearch.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_control = te_uas.drone_control:main',
            'door_control = te_uas.door_control:main',
            'reel_control = te_uas.reel_control:main',
            'logger = te_uas.logger:main',
            'killswitch = te_uas.killswitch:main',
            'test1 = te_uas.test1:main',
            'test2 = te_uas.test2:main',
            'test3 = te_uas.test3:main',
            'test4 = te_uas.test4:main',
            'test5 = te_uas.test5:main',
            'test6 = te_uas.test6:main',
            'global_control = te_uas.bone_control:main',
        ],
    },
)
