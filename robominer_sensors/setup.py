import os
from glob import glob
from setuptools import setup

package_name = 'robominer_sensors'

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
    maintainer='Jaan Rebane',
    maintainer_email='jaan.rebane@ttu.ee',
    description='Robominer sensors package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_sensor = robominer_sensors.temperature_sensor:main'
        ],
    },
)
