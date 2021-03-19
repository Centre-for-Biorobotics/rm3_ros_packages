import os
from glob import glob
from setuptools import setup

package_name = 'robominer_locomotion_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Roza Gkliva',
    maintainer_email='roza.gkliva@ttu.ee',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_steering = robominer_locomotion_control.open_loop_steering:main'
        ],
    },
)
