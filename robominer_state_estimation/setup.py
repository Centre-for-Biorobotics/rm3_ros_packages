import os
from glob import glob
from setuptools import setup

package_name = 'robominer_state_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Walid Remmas',
    maintainer_email='walid.remmas@taltech.ee',
    description='Package for robot state estimation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_dynamics = robominer_state_estimation.forward_dynamics:main',
            'rm3_inverse_kinematics = robominer_state_estimation.rm3_inverse_kinematics:main',
        ],
    },
)
