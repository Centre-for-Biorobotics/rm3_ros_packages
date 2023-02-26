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
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
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
            'rm3_forward_dynamics = robominer_state_estimation.rm3_forward_dynamics:main',
            'rm3_inverse_dynamics = robominer_state_estimation.rm3_inverse_dynamics:main',
            'rm3_inverse_kinematics = robominer_state_estimation.rm3_inverse_kinematics:main',
            'rm3_inverse_kinematics_forced = robominer_state_estimation.rm3_inverse_kinematics_forced:main',
            'messages_preparer_for_filtering = robominer_state_estimation.messages_preparer_for_filtering:main',
            'rm3_forward_kinematics = robominer_state_estimation.rm3_forward_kinematics:main',
        ],
    },
)
