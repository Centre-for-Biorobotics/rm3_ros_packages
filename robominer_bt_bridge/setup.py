import os
from setuptools import setup
from glob import glob

package_name = 'robominer_bt_bridge'

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
    description='Contains nodes that serve as interface between the low-level and BT',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_action_client = robominer_bt_bridge.move_action_client:main',
            'move_action_server = robominer_bt_bridge.move_action_server:main',
        ],
    },
)
