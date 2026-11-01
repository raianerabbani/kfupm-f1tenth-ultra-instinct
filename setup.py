from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kfupm_f1tenth_ultra_instinct'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'racelines'), glob('racelines/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Ultra Instinct',
    maintainer_email='raian@kfupm.edu.sa',
    description='Team Ultra Instinct - CIE-KFUPM F1TENTH Autonomous Racing Controller',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = kfupm_f1tenth_ultra_instinct.controller_node:main',
        ],
    },
)
